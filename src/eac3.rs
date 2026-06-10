//! Minimal E-AC-3 (Enhanced AC-3 / Dolby Digital Plus) bitstream parsing — the foundation
//! for transcoding ffmpeg's E-AC-3 core and injecting Atmos (OAMD + JOC) metadata.
//!
//! Syncframe/BSI field order per ETSI TS 102 366 / ATSC A/52 Annex E, cross-checked against
//! VoidXH/Cavern's `EnhancedAC3Header.Decode()`. NOTE: Cavern is under a non-commercial
//! licence (see convert-poc/PROGRESS.md) — this is for personal/local use; attribution kept.

use std::io::{self, Read, Write};

const SYNCWORD: [u8; 2] = [0x0B, 0x77];
/// numblkscod -> number of audio blocks per syncframe.
const NUM_BLOCKS: [u8; 4] = [1, 2, 3, 6];
/// fscod -> sample rate (Hz); 3 = reduced-rate (handled separately).
const SAMPLE_RATES: [u32; 4] = [48000, 44100, 32000, 0];

/// MSB-first bit reader over a byte slice.
struct BitReader<'a> {
    data: &'a [u8],
    /// Current position, in bits from the start of `data`.
    pos: usize,
}

impl<'a> BitReader<'a> {
    fn new(data: &'a [u8], byte_offset: usize) -> Self {
        Self {
            data,
            pos: byte_offset * 8,
        }
    }

    /// Position the reader at an absolute bit offset.
    fn seek(&mut self, bit: usize) {
        self.pos = bit;
    }

    /// Read `n` bits (n <= 32), MSB-first. Reads past the end yield zero bits.
    fn read(&mut self, n: u32) -> u32 {
        let mut v = 0u32;
        for _ in 0..n {
            let byte = self.pos >> 3;
            let bit = 7 - (self.pos & 7);
            let b = if byte < self.data.len() {
                (self.data[byte] >> bit) & 1
            } else {
                0
            };
            v = (v << 1) | b as u32;
            self.pos += 1;
        }
        v
    }
}

/// Parsed header of one E-AC-3 syncframe (only the fields we need to walk + classify frames).
#[derive(Debug, Clone, PartialEq)]
pub struct FrameInfo {
    /// Byte offset of the syncword in the stream.
    pub offset: usize,
    /// Total frame length in bytes.
    pub size: usize,
    /// Stream type: 0 = independent, 1 = dependent, 2 = AC-3-converted, 3 = reserved.
    pub strmtyp: u8,
    pub substreamid: u8,
    /// Number of audio blocks (1/2/3/6).
    pub blocks: u8,
    pub fscod: u8,
    pub sample_rate: u32,
    /// Channel mode (acmod).
    pub acmod: u8,
    pub lfe: bool,
    /// Bitstream id / decoder version: 16 = E-AC-3, <=8 = AC-3.
    pub bsid: u8,
}

impl FrameInfo {
    /// Full-bandwidth channel count (excludes LFE) implied by acmod.
    pub fn full_channels(&self) -> u8 {
        match self.acmod {
            0 => 2, // 1+1 dual mono
            1 => 1, // 1/0 C
            2 => 2, // 2/0 L R
            3 => 3, // 3/0 L C R
            4 => 3, // 2/1 L R S
            5 => 4, // 3/1 L C R S
            6 => 4, // 2/2 L R Ls Rs
            7 => 5, // 3/2 L C R Ls Rs
            _ => 0,
        }
    }

    /// Samples represented by this frame (blocks * 256).
    pub fn samples(&self) -> u32 {
        self.blocks as u32 * 256
    }
}

/// Decode one syncframe's [`FrameInfo`], assuming a syncword at `off` with at least 6 bytes
/// available from there. Shared by [`parse_frames`] and the streaming [`transform_frames_io`].
fn read_frame_header(data: &[u8], off: usize) -> FrameInfo {
    let mut r = BitReader::new(data, off);
    let _sync = r.read(16);
    let strmtyp = r.read(2) as u8;
    let substreamid = r.read(3) as u8;
    let words = r.read(11) as usize + 1; // words_per_syncframe
    let size = words * 2;
    let fscod = r.read(2) as u8;
    let numblkscod = r.read(2) as usize;
    let acmod = r.read(3) as u8;
    let lfe = r.read(1) == 1;
    let bsid = r.read(5) as u8;
    let blocks = if fscod == 3 {
        6
    } else {
        NUM_BLOCKS[numblkscod]
    };
    let sample_rate = SAMPLE_RATES[fscod as usize];
    FrameInfo {
        offset: off,
        size,
        strmtyp,
        substreamid,
        blocks,
        fscod,
        sample_rate,
        acmod,
        lfe,
        bsid,
    }
}

/// Parse every E-AC-3 syncframe in a raw elementary stream.
pub fn parse_frames(data: &[u8]) -> Vec<FrameInfo> {
    let mut frames = Vec::new();
    let mut off = 0usize;
    while off + 6 <= data.len() {
        if data[off..off + 2] != SYNCWORD {
            match find_sync(data, off + 1) {
                Some(p) => {
                    off = p;
                    continue;
                }
                None => break,
            }
        }

        let info = read_frame_header(data, off);
        let size = info.size;
        frames.push(info);

        if size == 0 {
            break; // malformed; avoid infinite loop
        }
        off += size;
    }
    frames
}

fn find_sync(data: &[u8], from: usize) -> Option<usize> {
    (from..data.len().saturating_sub(1)).find(|&i| data[i..i + 2] == SYNCWORD)
}

// ---------------------------------------------------------------------------
// Aux-data injection: grow each E-AC-3 frame and write an EMDF payload into the
// frame's auxiliary-data field, recomputing crc2. This is how DD+ Atmos rides
// in an E-AC-3 stream (OAMD + JOC live in the EMDF carried here).
//
// Frame tail layout (ATSC A/52 §5.4.4–5.4.5, Annex E), reading from frame end:
//   [crc2:16][crcrsv:1][auxdatae:1][auxdatal:14][auxbits: auxdatal bits] ...
// auxbits user data sits at the *end* of the aux field (forward order); a
// decoder reads auxdatal at a fixed offset from the end and backs up that many
// bits. We exploit this: keep the original body bytes verbatim (only patching
// frmsiz), append a zero gap, then the payload + trailing fields. The audio
// bits are untouched, so the decoded PCM is bit-identical.
// ---------------------------------------------------------------------------

/// E-AC-3 frame max size in bytes (frmsiz is 11 bits → up to 2048 words).
const MAX_FRAME_BYTES: usize = 2048 * 2;

/// MSB-first bit writer.
struct BitWriter {
    out: Vec<u8>,
    cur: u8,
    nbits: u8,
}

impl BitWriter {
    fn with_capacity(cap: usize) -> Self {
        Self {
            out: Vec::with_capacity(cap),
            cur: 0,
            nbits: 0,
        }
    }

    #[inline]
    fn bit(&mut self, b: u8) {
        self.cur = (self.cur << 1) | (b & 1);
        self.nbits += 1;
        if self.nbits == 8 {
            self.out.push(self.cur);
            self.cur = 0;
            self.nbits = 0;
        }
    }

    /// Write the low `n` bits of `val`, MSB-first.
    fn write(&mut self, val: u32, n: u32) {
        for i in (0..n).rev() {
            self.bit(((val >> i) & 1) as u8);
        }
    }

    /// Copy `nbits` MSB-first bits from `data` (must hold at least `nbits` bits).
    fn copy_bits(&mut self, data: &[u8], nbits: usize) {
        let mut i = 0;
        if self.nbits == 0 {
            // Byte-aligned fast path for the whole-byte prefix.
            let full = nbits >> 3;
            self.out.extend_from_slice(&data[..full]);
            i = full * 8;
        }
        while i < nbits {
            let b = (data[i >> 3] >> (7 - (i & 7))) & 1;
            self.bit(b);
            i += 1;
        }
    }

    /// Copy `nbits` MSB-first bits from `data` starting at absolute bit offset `start`.
    fn copy_bits_from(&mut self, data: &[u8], start: usize, nbits: usize) {
        for i in start..start + nbits {
            self.bit((data[i >> 3] >> (7 - (i & 7))) & 1);
        }
    }

    fn bits_written(&self) -> usize {
        self.out.len() * 8 + self.nbits as usize
    }

    fn finish(self) -> Vec<u8> {
        debug_assert_eq!(self.nbits, 0, "frame not byte-aligned at finish");
        self.out
    }
}

/// E-AC-3 crc2: poly 0x8005, init 0, MSB-first, non-augmented.
/// Verified to match ffmpeg's own output byte-for-byte (see convert-poc/tools/crc_probe.py).
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u32 = 0;
    for &byte in data {
        for i in (0..8).rev() {
            let b = ((byte >> i) & 1) as u32;
            let top = (crc >> 15) & 1;
            crc = if b != top {
                ((crc << 1) ^ 0x8005) & 0xFFFF
            } else {
                (crc << 1) & 0xFFFF
            };
        }
    }
    crc as u16
}

/// Inject `payload` (MSB-first, `payload_bits` significant bits) into a frame's
/// aux-data field, growing the frame and recomputing crc2. The input `frame`
/// must currently have `auxdatae == 0` (true for ffmpeg-encoded frames).
///
/// Returns the new, larger frame. The audio blocks are preserved bit-exactly.
pub fn inject_aux(frame: &[u8], payload: &[u8], payload_bits: usize) -> Vec<u8> {
    let old_size = frame.len();
    debug_assert!(old_size >= 6);
    // Body = whole frame minus the trailing auxdatae(1)+crcrsv(1)+crc2(16) = 18 bits.
    let body_bits = old_size * 8 - 18;
    // After the body we write: gap(G) + auxbits(P) + auxdatal(14) + auxdatae(1)
    // + crcrsv(1) + crc2(16). Choose the smallest even byte count NS that fits
    // with G >= 0, then G fills the slack (< 16 bits).
    let need_bits = body_bits + payload_bits + 14 + 1 + 1 + 16;
    let mut ns = need_bits.div_ceil(8);
    if ns % 2 != 0 {
        ns += 1;
    }
    assert!(
        ns <= MAX_FRAME_BYTES,
        "injected frame {ns} B exceeds E-AC-3 max {MAX_FRAME_BYTES} B (payload too large)"
    );
    let gap = ns * 8 - (body_bits + payload_bits + 14 + 1 + 1 + 16);

    // Patch frmsiz (11 bits at bit offset 21 = low 3 bits of byte 2 + all of byte 3).
    let new_words = (ns / 2) as u32;
    let frmsiz_val = new_words - 1;
    let mut body = frame.to_vec();
    body[2] = (body[2] & 0xF8) | ((frmsiz_val >> 8) & 0x07) as u8;
    body[3] = (frmsiz_val & 0xFF) as u8;

    let mut w = BitWriter::with_capacity(ns);
    w.copy_bits(&body, body_bits); // sync + bsi + audfrm + audblks + original padding
    for _ in 0..gap {
        w.bit(0);
    }
    w.copy_bits(payload, payload_bits); // auxbits (user data)
    w.write(payload_bits as u32, 14); // auxdatal (length in bits)
    w.bit(1); // auxdatae
    w.bit(0); // crcrsv

    debug_assert_eq!(w.bits_written(), ns * 8 - 16, "pre-crc misalignment");
    debug_assert_eq!(w.out.len(), ns - 2);
    let crc = crc16(&w.out[2..]); // coverage: everything after the syncword
    w.write(crc as u32, 16);

    let out = w.finish();
    debug_assert_eq!(out.len(), ns);
    out
}

/// Read the user aux payload from a frame, if present. Returns (bytes, nbits),
/// MSB-first, matching what `inject_aux` wrote. Round-trip self-check helper.
pub fn read_aux(frame: &[u8]) -> Option<(Vec<u8>, usize)> {
    let total = frame.len() * 8;
    if total < 32 {
        return None;
    }
    let mut r = BitReader::new(frame, 0);
    // auxdatae is the bit just before crcrsv+crc2 (i.e. at total-18).
    r.seek(total - 18);
    let auxdatae = r.read(1);
    if auxdatae == 0 {
        return None;
    }
    // auxdatal: the 14 bits immediately preceding auxdatae (total-32 .. total-18).
    r.seek(total - 32);
    let auxdatal = r.read(14) as usize;
    if auxdatal == 0 || auxdatal > total.saturating_sub(32) {
        return None;
    }
    // Payload: auxdatal bits ending just before the auxdatal field.
    let start = total - 32 - auxdatal;
    r.seek(start);
    let nbytes = auxdatal.div_ceil(8);
    let mut bytes = vec![0u8; nbytes];
    let mut bw = BitWriter::with_capacity(nbytes);
    for _ in 0..auxdatal {
        bw.bit(r.read(1) as u8);
    }
    // Flush any partial final byte left-aligned (MSB-first), mirroring the writer.
    if bw.nbits != 0 {
        bw.cur <<= 8 - bw.nbits;
        bw.out.push(bw.cur);
    }
    bytes[..bw.out.len()].copy_from_slice(&bw.out);
    Some((bytes, auxdatal))
}

/// Verbose BSI trace: print every bsi() field of one frame. For diffing my ffmpeg core against a
/// real Dolby core (mixing_metadata / info_metadata presence, etc.).
pub fn bsi_dump(frame: &[u8], info: &FrameInfo) {
    if info.bsid != 16 {
        println!("  not E-AC-3 (bsid={})", info.bsid);
        return;
    }
    let cm = info.acmod as u32;
    let lfe = info.lfe;
    let blocks = info.blocks as u32;
    let fscod = info.fscod as u32;
    let strmtyp = info.strmtyp;
    println!("  strmtyp={strmtyp} acmod={cm} lfe={lfe} blocks={blocks} fscod={fscod}");

    let mut r = BitReader::new(frame, 0);
    r.seek(45);
    let dialnorm = r.read(5);
    let compre = r.read(1);
    if compre == 1 {
        r.read(8);
    }
    println!("  dialnorm={dialnorm} compre={compre}");
    if cm == 0 {
        r.read(5);
        if r.read(1) == 1 {
            r.read(8);
        }
    }
    if strmtyp == 1 && r.read(1) == 1 {
        r.read(16);
    }
    let mixmdate = r.read(1);
    print!("  mixmdate={mixmdate}");
    if mixmdate == 1 {
        if cm > 2 {
            r.read(2);
        }
        if (cm & 1) != 0 && cm > 2 {
            r.read(6);
        }
        if (cm & 0x4) != 0 {
            r.read(6);
        }
        if lfe && r.read(1) == 1 {
            r.read(5);
        }
        if strmtyp == 0 {
            let pgmscle = r.read(1);
            if pgmscle == 1 {
                r.read(6);
            }
            if cm == 0 && r.read(1) == 1 {
                r.read(6);
            }
            let extpgmscle = r.read(1);
            if extpgmscle == 1 {
                r.read(6);
            }
            let mixdef = r.read(2);
            print!(" pgmscle={pgmscle} extpgmscle={extpgmscle} mixdef={mixdef}");
            match mixdef {
                1 => {
                    r.read(5);
                }
                2 => {
                    r.read(12);
                }
                3 => {
                    let n = r.read(5) + 2;
                    for _ in 0..n {
                        r.read(8);
                    }
                }
                _ => {}
            }
            if cm < 2 {
                if r.read(1) == 1 {
                    r.read(8);
                    r.read(6);
                }
                if cm == 0 && r.read(1) == 1 {
                    r.read(8);
                    r.read(6);
                }
            }
            let frmmixcfginfoe = r.read(1);
            print!(" frmmixcfginfoe={frmmixcfginfoe}");
            if frmmixcfginfoe == 1 {
                if blocks == 1 {
                    r.read(5);
                } else {
                    for _ in 0..blocks {
                        if r.read(1) == 1 {
                            r.read(5);
                        }
                    }
                }
            }
        }
    }
    println!();
    let infomdate = r.read(1);
    print!("  infomdate={infomdate}");
    if infomdate == 1 {
        let bsmod = r.read(3);
        r.read(1);
        r.read(1);
        print!(" bsmod={bsmod}");
        if cm == 2 {
            r.read(2);
            r.read(2);
        } else if cm >= 6 {
            let dsurexmod = r.read(2);
            print!(" dsurexmod={dsurexmod}");
        }
        let audprodie = r.read(1);
        print!(" audprodie={audprodie}");
        if audprodie == 1 {
            r.read(5);
            r.read(2);
            r.read(1);
        }
        if cm == 0 && r.read(1) == 1 {
            r.read(5);
            r.read(2);
            r.read(1);
        }
        if fscod < 3 {
            r.read(1);
        }
    }
    println!();
    if strmtyp == 0 && blocks != 6 {
        r.read(1);
    }
    let addbsie = r.read(1);
    print!("  addbsie={addbsie}");
    if addbsie == 1 {
        let addbsil = r.read(6);
        print!(" addbsil={addbsil} bytes=[");
        for _ in 0..(addbsil + 1) {
            print!("{:02x} ", r.read(8));
        }
        print!("]");
    }
    println!();
    // audfrm: skipFieldSyntaxEnabled sits at a fixed offset (audfrm_start + 10) for blocks==6
    // independent frames — readable even when full audfrm parse bails on coupling.
    let audfrm_start = r.pos;
    let mut r2 = BitReader::new(frame, 0);
    r2.seek(audfrm_start + 10);
    let skipfld = r2.read(1);
    // auxdatae is the bit 18 from the end (before crcrsv+crc2).
    let total = frame.len() * 8;
    let mut r3 = BitReader::new(frame, 0);
    r3.seek(total - 18);
    let auxdatae = r3.read(1);
    println!("  audfrm: skipFieldSyntaxEnabled={skipfld}  |  tail: auxdatae={auxdatae}");
}

/// Locate the `infomdate` flag bit (start of `info_metadata()`) by walking bsi() up to that point.
/// Returns `(bit_index_of_infomdate, infomdate_value)`. Independent E-AC-3 (strmtyp 0) only.
fn bsi_infomdate_pos(frame: &[u8], info: &FrameInfo) -> Option<(usize, u32)> {
    if info.bsid != 16 {
        return None;
    }
    let cm = info.acmod as u32;
    let lfe = info.lfe;
    let blocks = info.blocks as u32;
    let strmtyp = info.strmtyp;

    let mut r = BitReader::new(frame, 0);
    r.seek(45);
    r.read(5); // dialnorm
    if r.read(1) == 1 {
        r.read(8); // compr
    }
    if cm == 0 {
        r.read(5);
        if r.read(1) == 1 {
            r.read(8);
        }
    }
    if strmtyp == 1 && r.read(1) == 1 {
        r.read(16);
    }
    // mixing_metadata()
    if r.read(1) == 1 {
        if cm > 2 {
            r.read(2);
        }
        if (cm & 1) != 0 && cm > 2 {
            r.read(6);
        }
        if (cm & 0x4) != 0 {
            r.read(6);
        }
        if lfe && r.read(1) == 1 {
            r.read(5);
        }
        if strmtyp == 0 {
            if r.read(1) == 1 {
                r.read(6);
            }
            if cm == 0 && r.read(1) == 1 {
                r.read(6);
            }
            if r.read(1) == 1 {
                r.read(6);
            }
            match r.read(2) {
                1 => {
                    r.read(5);
                }
                2 => {
                    r.read(12);
                }
                3 => {
                    let n = r.read(5) + 2;
                    for _ in 0..n {
                        r.read(8);
                    }
                }
                _ => {}
            }
            if cm < 2 {
                if r.read(1) == 1 {
                    r.read(8);
                    r.read(6);
                }
                if cm == 0 && r.read(1) == 1 {
                    r.read(8);
                    r.read(6);
                }
            }
            if r.read(1) == 1 {
                if blocks == 1 {
                    r.read(5);
                } else {
                    for _ in 0..blocks {
                        if r.read(1) == 1 {
                            r.read(5);
                        }
                    }
                }
            }
        }
    }
    let q = r.pos;
    let infomdate = r.read(1);
    Some((q, infomdate))
}

/// Locate the `addbsie` flag bit in an E-AC-3 syncframe by walking `bsi()` (ATSC A/52 Annex E,
/// cross-checked vs Cavern `BitStreamInformation.cs` / `Mixing.cs` / `Informational.cs`).
/// Returns `(bit_index_of_addbsie, addbsie_value)`. Independent/dependent E-AC-3 only.
fn bsi_addbsie_pos(frame: &[u8], info: &FrameInfo) -> Option<(usize, u32)> {
    if info.bsid != 16 {
        return None; // E-AC-3 only
    }
    let cm = info.acmod as u32;
    let lfe = info.lfe;
    let blocks = info.blocks as u32;
    let fscod = info.fscod as u32;
    let strmtyp = info.strmtyp;

    let mut r = BitReader::new(frame, 0);
    r.seek(45); // sync16+strmtyp2+substreamid3+frmsiz11+fscod2+numblkscod2+acmod3+lfe1+bsid5

    r.read(5); // dialnorm
    if r.read(1) == 1 {
        r.read(8); // compr
    }
    if cm == 0 {
        r.read(5);
        if r.read(1) == 1 {
            r.read(8);
        }
    }
    if strmtyp == 1 && r.read(1) == 1 {
        r.read(16); // dependent: chanmap
    }
    // mixing_metadata()
    if r.read(1) == 1 {
        if cm > 2 {
            r.read(2); // dmixmod
        }
        if (cm & 1) != 0 && cm > 2 {
            r.read(6); // centerDownmix
        }
        if (cm & 0x4) != 0 {
            r.read(6); // surroundDownmix
        }
        if lfe && r.read(1) == 1 {
            r.read(5); // lfemixlevcod
        }
        if strmtyp == 0 {
            if r.read(1) == 1 {
                r.read(6); // pgmscl
            }
            if cm == 0 && r.read(1) == 1 {
                r.read(6); // pgmscl2
            }
            if r.read(1) == 1 {
                r.read(6); // extpgmscl
            }
            match r.read(2) {
                1 => {
                    r.read(5);
                }
                2 => {
                    r.read(12);
                }
                3 => {
                    let n = r.read(5) + 2;
                    for _ in 0..n {
                        r.read(8);
                    }
                }
                _ => {}
            }
            if cm < 2 {
                if r.read(1) == 1 {
                    r.read(8);
                    r.read(6);
                }
                if cm == 0 && r.read(1) == 1 {
                    r.read(8);
                    r.read(6);
                }
            }
            if r.read(1) == 1 {
                // frmmixcfginfoe
                if blocks == 1 {
                    r.read(5);
                } else {
                    for _ in 0..blocks {
                        if r.read(1) == 1 {
                            r.read(5);
                        }
                    }
                }
            }
        }
    }
    // info_metadata()
    if r.read(1) == 1 {
        r.read(3); // bsmod
        r.read(1); // copyright
        r.read(1); // original
        if cm == 2 {
            r.read(2);
            r.read(2);
        } else if cm >= 6 {
            r.read(2);
        }
        if r.read(1) == 1 {
            r.read(5);
            r.read(2);
            r.read(1);
        }
        if cm == 0 && r.read(1) == 1 {
            r.read(5);
            r.read(2);
            r.read(1);
        }
        if fscod < 3 {
            r.read(1); // sourcefscod
        }
    }
    if strmtyp == 0 && blocks != 6 {
        r.read(1); // convsync
    }
    let p = r.pos;
    let addbsie = r.read(1);
    Some((p, addbsie))
}

/// Inject BOTH the addbsi Atmos flag (front, in `bsi`) and an aux EMDF payload (tail) into one
/// frame in a single grow + crc2 pass. `frame` must currently have `addbsie == 0` (ffmpeg output).
/// Audio blocks are preserved bit-exactly. `addbsi` is the raw addbsi field bytes.
pub fn inject_frame_full(
    frame: &[u8],
    info: &FrameInfo,
    addbsi: &[u8],
    aux: &[u8],
    aux_bits: usize,
) -> Vec<u8> {
    let l = frame.len() * 8;
    let (p, addbsie) = bsi_addbsie_pos(frame, info).expect("E-AC-3 BSI parse");
    assert_eq!(
        addbsie, 0,
        "frame already carries addbsi (addbsie=1) — unsupported"
    );
    let addbsi_bits = addbsi.len() * 8;

    // Pre-crc bit budget (derivation in convert-poc notes): l + addbsi_bits + aux_bits + gap + 4.
    let fixed = l + addbsi_bits + aux_bits + 4 + 16; // + crc2(16), gap = 0
    let mut ns = fixed.div_ceil(8);
    if ns % 2 != 0 {
        ns += 1;
    }
    assert!(
        ns <= MAX_FRAME_BYTES,
        "injected frame {ns} B exceeds E-AC-3 max {MAX_FRAME_BYTES} B"
    );
    let gap = ns * 8 - fixed;

    // Patch frmsiz in a copy of the front bytes.
    let new_words = (ns / 2) as u32;
    let frmsiz_val = new_words - 1;
    let mut fr = frame.to_vec();
    fr[2] = (fr[2] & 0xF8) | ((frmsiz_val >> 8) & 0x07) as u8;
    fr[3] = (frmsiz_val & 0xFF) as u8;

    let mut w = BitWriter::with_capacity(ns);
    w.copy_bits(&fr, p); // sync + bsi up to (not incl.) addbsie, frmsiz patched
    w.bit(1); // addbsie = 1
    w.write(addbsi.len() as u32 - 1, 6); // addbsil
    w.copy_bits(addbsi, addbsi_bits); // addbsi field
    w.copy_bits_from(frame, p + 1, l - 18 - (p + 1)); // audfrm + audblks + original padding
    for _ in 0..gap {
        w.bit(0);
    }
    w.copy_bits(aux, aux_bits); // EMDF payload
    w.write(aux_bits as u32, 14); // auxdatal
    w.bit(1); // auxdatae
    w.bit(0); // crcrsv

    debug_assert_eq!(w.bits_written(), ns * 8 - 16);
    let crc = crc16(&w.out[2..]);
    w.write(crc as u32, 16);
    let out = w.finish();
    debug_assert_eq!(out.len(), ns);
    out
}

/// Inject the addbsi Atmos flag (FRONT, in bsi) **and** carry the EMDF in an audio-block **skip
/// field** (mid-frame) — the way real Dolby DD+ JOC streams do it. This enables
/// `skipFieldSyntaxEnabled` in audfrm and writes a `skiple` flag at every block's skip-field point
/// (the EMDF rides in `target_block`; the rest get `skiple=0`). Returns None if the frame's audio
/// blocks use features outside our supported subset (caller may fall back to aux carriage).
///
/// `emdf` must be ≤ 511 bytes (skipl is 9 bits). The audio mantissa data is copied verbatim, so the
/// decoded PCM is unchanged; only frmsiz and crc2 are recomputed.
pub fn inject_frame_skipfield(
    frame: &[u8],
    info: &FrameInfo,
    addbsi: &[u8],
    emdf: &[u8],
    target_block: usize,
) -> Option<Vec<u8>> {
    assert!(
        emdf.len() <= 511,
        "EMDF {} B exceeds skipl 9-bit max",
        emdf.len()
    );
    let l = frame.len() * 8;
    let (p, addbsie) = bsi_addbsie_pos(frame, info)?;
    if addbsie != 0 {
        return None; // already carries addbsi
    }
    let bsi_end = p + 1; // audfrm start (addbsie == 0)
    let skipfld_bit = crate::eac3_audblk::skipfld_enable_bit(frame, info, bsi_end)?;
    let (points, _end) = crate::eac3_audblk::skip_points(frame, info, bsi_end)?;
    if points.len() != 6 || target_block >= 6 {
        return None;
    }
    // Edit points must be strictly increasing: addbsie < skipfld < block skip points.
    if !(p < skipfld_bit && skipfld_bit < points[0]) {
        return None;
    }

    // If the core lacks info_metadata (ffmpeg emits infomdate=0; real Dolby Atmos cores set
    // infomdate=1), synthesize an all-zero info_metadata() so the bsi matches a Dolby-authored core.
    // The 9-bit body (bsmod3·copyright1·original1·dsurexmod2·audprodie1·sourcefscod1) is valid only
    // for acmod 7 @ fscod<3 — exactly our 5.1 core config.
    let (info_pos, info_val) = bsi_infomdate_pos(frame, info)?;
    let insert_info = info_val == 0 && info.acmod == 7 && info.fscod < 3 && info_pos < p;
    let info_extra = if insert_info { 9 } else { 0 };

    let addbsi_bits = addbsi.len() * 8;
    let emdf_bits = emdf.len() * 8;
    // New body bits (sync..padding, excluding the 18-bit tail), after all insertions:
    //   info_metadata synthesis (optional): +9 bits
    //   addbsie replacement: 1+6+addbsi_bits replaces the single addbsie bit → +(6+addbsi_bits)
    //   skipFieldSyntaxEnabled: flip 0→1, no size change
    //   six skiple fields: five `skiple=0` (1 bit) + target `skiple=1`+skipl(9)+emdf
    let skip_extra = 5 * 1 + (1 + 9 + emdf_bits);
    let new_body = (l - 18) + info_extra + (6 + addbsi_bits) + skip_extra;
    let fixed = new_body + 2 + 16; // auxdatae(0) + crcrsv(0) + crc2(16)
    let mut ns = fixed.div_ceil(8);
    if ns % 2 != 0 {
        ns += 1;
    }
    if ns > MAX_FRAME_BYTES {
        return None;
    }
    let gap = ns * 8 - fixed;

    // Patch frmsiz in a copy of the front bytes.
    let new_words = (ns / 2) as u32;
    let frmsiz_val = new_words - 1;
    let mut fr = frame.to_vec();
    fr[2] = (fr[2] & 0xF8) | ((frmsiz_val >> 8) & 0x07) as u8;
    fr[3] = (frmsiz_val & 0xFF) as u8;

    let mut w = BitWriter::with_capacity(ns);
    if insert_info {
        // [0, info_pos): bsi up to the infomdate flag, frmsiz patched.
        w.copy_bits(&fr, info_pos);
        w.bit(1); // infomdate = 1
        w.write(0, 9); // info_metadata() body, all zero (matches Dolby Atmos core)
        // (info_pos+1, p): bits between old infomdate flag and addbsie (e.g. convsync; empty here).
        w.copy_bits_from(&fr, info_pos + 1, p - (info_pos + 1));
    } else {
        // [0, p): sync + bsi up to (not incl.) addbsie, frmsiz patched.
        w.copy_bits(&fr, p);
    }
    // addbsi field (addbsie=1, addbsil, bytes).
    w.bit(1);
    w.write(addbsi.len() as u32 - 1, 6);
    w.copy_bits(addbsi, addbsi_bits);
    // (p+1, skipfld_bit): rest of bsi + audfrm up to the skip-field-enable flag.
    w.copy_bits_from(frame, p + 1, skipfld_bit - (p + 1));
    // skipFieldSyntaxEnabled = 1 (replaces the original 0 bit).
    w.bit(1);
    // (skipfld_bit+1, points[0]): rest of audfrm + start of block 0.
    w.copy_bits_from(frame, skipfld_bit + 1, points[0] - (skipfld_bit + 1));
    // Each block's skip field, then the block body up to the next skip point.
    for b in 0..6 {
        if b == target_block {
            w.bit(1); // skiple = 1
            w.write(emdf.len() as u32, 9); // skipl (bytes)
            w.copy_bits(emdf, emdf_bits);
        } else {
            w.bit(0); // skiple = 0
        }
        let seg_end = if b + 1 < 6 { points[b + 1] } else { l - 18 };
        w.copy_bits_from(frame, points[b], seg_end - points[b]);
    }
    // Tail: padding, then auxdatae=0, crcrsv=0, crc2.
    for _ in 0..gap {
        w.bit(0);
    }
    w.bit(0); // auxdatae = 0
    w.bit(0); // crcrsv = 0
    debug_assert_eq!(w.bits_written(), ns * 8 - 16);
    let crc = crc16(&w.out[2..]);
    w.write(crc as u32, 16);
    let out = w.finish();
    debug_assert_eq!(out.len(), ns);
    Some(out)
}

/// Definitive-test surgery: replace a REAL Dolby core's native EMDF (in its skip field) with our
/// own `my_emdf`, in place, **without** parsing the (possibly coupled) audio blocks. We treat the
/// whole frame as opaque bit-copy except: (1) the addbsi complexity byte (patched to
/// `new_complexity`), and (2) the `native_len`-byte EMDF container at `emdf_bit` (overwritten with
/// `my_emdf` padded with zeros to `native_len`). Frame size is unchanged; only crc2 is recomputed.
/// The coupled audio data is preserved bit-exact. Returns None if `my_emdf` doesn't fit.
pub fn splice_emdf_into_core(
    frame: &[u8],
    info: &FrameInfo,
    emdf_bit: usize,
    native_len: usize,
    my_emdf: &[u8],
    new_complexity: Option<u8>,
) -> Option<Vec<u8>> {
    if my_emdf.len() > native_len {
        return None;
    }
    let total = frame.len() * 8;
    let after = emdf_bit + native_len * 8;
    if after > total {
        return None;
    }
    let (p, addbsie) = bsi_addbsie_pos(frame, info)?;
    if addbsie != 1 {
        return None;
    }
    // complexity byte = addbsie(1) + addbsil(6) + addbsi byte0(8) → second addbsi byte.
    let complexity_bit = p + 1 + 6 + 8;
    if complexity_bit + 8 > emdf_bit {
        return None;
    }

    let mut w = BitWriter::with_capacity(frame.len());
    let mut cur = 0usize;
    if let Some(cx) = new_complexity {
        w.copy_bits_from(frame, 0, complexity_bit);
        w.write(cx as u32, 8);
        cur = complexity_bit + 8;
    }
    w.copy_bits_from(frame, cur, emdf_bit - cur);
    w.copy_bits(my_emdf, my_emdf.len() * 8);
    for _ in 0..((native_len - my_emdf.len()) * 8) {
        w.bit(0);
    }
    w.copy_bits_from(frame, after, total - after);
    let mut out = w.finish();
    if out.len() != frame.len() {
        return None;
    }
    let n = out.len();
    let crc = crc16(&out[2..n - 2]);
    out[n - 2] = (crc >> 8) as u8;
    out[n - 1] = (crc & 0xFF) as u8;
    Some(out)
}

/// Read back the addbsi Atmos extension: `(flag_ec3_extension_type_a, complexity_index_type_a)`.
/// Assumes the §8.3 layout: reserved(7) · flag(1) · complexity(8).
pub fn read_addbsi(frame: &[u8], info: &FrameInfo) -> Option<(bool, u8)> {
    let (p, addbsie) = bsi_addbsie_pos(frame, info)?;
    if addbsie == 0 {
        return None;
    }
    let mut r = BitReader::new(frame, 0);
    r.seek(p + 1);
    let nbytes = r.read(6) + 1; // addbsil + 1
    if nbytes < 2 {
        return None;
    }
    let b0 = r.read(8);
    let flag = (b0 & 1) == 1;
    let complexity = r.read(8) as u8;
    Some((flag, complexity))
}

/// Raw addbsi field: `(addbsil+1, bytes)`. For comparing our injected addbsi vs real Dolby.
pub fn read_addbsi_raw(frame: &[u8], info: &FrameInfo) -> Option<(usize, Vec<u8>)> {
    let (p, addbsie) = bsi_addbsie_pos(frame, info)?;
    if addbsie == 0 {
        return None;
    }
    let mut r = BitReader::new(frame, 0);
    r.seek(p + 1);
    let nbytes = r.read(6) as usize + 1;
    let bytes = (0..nbytes).map(|_| r.read(8) as u8).collect();
    Some((nbytes, bytes))
}

/// Bit offset where the audio-frame (`audfrm`) data begins, i.e. the end of `bsi()` including any
/// `addbsi`. Used by the audio-block walker to locate the skip field.
pub fn audfrm_start_bit(frame: &[u8], info: &FrameInfo) -> Option<usize> {
    let (p, addbsie) = bsi_addbsie_pos(frame, info)?;
    if addbsie == 0 {
        return Some(p + 1);
    }
    let mut r = BitReader::new(frame, 0);
    r.seek(p + 1);
    let nbytes = r.read(6) + 1; // addbsil + 1
    Some(p + 1 + 6 + (nbytes as usize) * 8)
}

/// Walk every syncframe and inject a per-frame aux payload. `payload_for` is
/// called with each frame's `FrameInfo` and index; return `None` (or 0 bits) to
/// leave a frame unchanged. Inter-frame bytes (if any) are copied verbatim.
pub fn inject_stream<F>(data: &[u8], mut payload_for: F) -> Vec<u8>
where
    F: FnMut(&FrameInfo, usize) -> Option<(Vec<u8>, usize)>,
{
    let frames = parse_frames(data);
    let mut out = Vec::with_capacity(data.len() + data.len() / 8);
    let mut cursor = 0usize;
    for (i, f) in frames.iter().enumerate() {
        if f.offset > cursor {
            out.extend_from_slice(&data[cursor..f.offset]); // resync gap, verbatim
        }
        let frame = &data[f.offset..f.offset + f.size];
        match payload_for(f, i) {
            Some((payload, bits)) if bits > 0 => {
                out.extend_from_slice(&inject_aux(frame, &payload, bits));
            }
            _ => out.extend_from_slice(frame),
        }
        cursor = f.offset + f.size;
    }
    if cursor < data.len() {
        out.extend_from_slice(&data[cursor..]);
    }
    out
}

/// Streaming counterpart of [`inject_stream`], generalised to arbitrary per-frame transforms.
///
/// Reads an E-AC-3 elementary stream from `reader`, hands each whole syncframe to `process`
/// (`process(frame_info, global_frame_index, frame_bytes) -> replacement_bytes`), and writes the
/// result to `writer`. Resync gaps between frames and any trailing partial bytes are copied
/// verbatim. Only a bounded window (~4 MiB plus one frame) is ever held in memory, so memory use is
/// independent of file size. Returns the number of frames processed.
///
/// For the same input and an equivalent transform, the output is byte-for-byte identical to walking
/// [`parse_frames`] over the whole buffer and transforming each frame (see the tests).
pub fn transform_frames_io<R, W, F>(reader: R, writer: W, process: F) -> io::Result<usize>
where
    R: Read,
    W: Write,
    F: FnMut(&FrameInfo, usize, &[u8]) -> Vec<u8>,
{
    transform_frames_io_chunked(reader, writer, 1 << 22, process) // 4 MiB read granularity
}

fn transform_frames_io_chunked<R, W, F>(
    mut reader: R,
    mut writer: W,
    chunk: usize,
    mut process: F,
) -> io::Result<usize>
where
    R: Read,
    W: Write,
    F: FnMut(&FrameInfo, usize, &[u8]) -> Vec<u8>,
{
    let chunk = chunk.max(8);
    let mut buf: Vec<u8> = Vec::new();
    let mut eof = false;
    let mut frame_idx = 0usize;

    while !eof || !buf.is_empty() {
        // Pull another chunk from the reader (until EOF), appended to any leftover partial frame.
        if !eof {
            let start = buf.len();
            buf.resize(start + chunk, 0);
            let mut filled = start;
            while filled < buf.len() {
                match reader.read(&mut buf[filled..])? {
                    0 => {
                        eof = true;
                        break;
                    }
                    n => filled += n,
                }
            }
            buf.truncate(filled);
        }

        // Emit every complete frame (and verbatim resync gap) the buffer currently holds.
        let mut pos = 0usize;
        loop {
            if pos + 2 > buf.len() {
                break; // need more bytes to test the syncword
            }
            if buf[pos..pos + 2] != SYNCWORD {
                match find_sync(&buf, pos) {
                    Some(p) => {
                        writer.write_all(&buf[pos..p])?; // resync gap, verbatim
                        pos = p;
                        continue;
                    }
                    None => break,
                }
            }
            if pos + 6 > buf.len() {
                break; // need the 6-byte header to learn the frame size
            }
            let mut info = read_frame_header(&buf, pos);
            info.offset = 0; // `frame_bytes` is slice-relative; an absolute offset is meaningless
            let size = info.size;
            if size == 0 {
                // Unreachable for valid streams (words+1 >= 1 -> size >= 2); resync defensively.
                match find_sync(&buf, pos + 1) {
                    Some(p) => {
                        writer.write_all(&buf[pos..p])?;
                        pos = p;
                        continue;
                    }
                    None => break,
                }
            }
            if pos + size > buf.len() {
                break; // frame not fully buffered yet
            }
            let outframe = process(&info, frame_idx, &buf[pos..pos + size]);
            writer.write_all(&outframe)?;
            frame_idx += 1;
            pos += size;
        }

        // At EOF, whatever is left (incomplete final frame / trailing bytes) is copied verbatim,
        // matching `inject_stream`'s trailing copy.
        if eof && pos < buf.len() {
            writer.write_all(&buf[pos..])?;
            pos = buf.len();
        }
        buf.drain(..pos);
    }

    writer.flush()?;
    Ok(frame_idx)
}

/// Stream `reader` and return the [`FrameInfo`] of every complete syncframe **without retaining the
/// frame bytes** — the grid-building counterpart to [`transform_frames_io`]. It walks frames with the
/// identical logic (same resync, same "a truncated trailing frame is not a frame"), so the frame at
/// index `i` here is exactly the frame `transform_frames_io` delivers at index `i`; the two are a
/// matched pair and must stay in lockstep. Only a bounded ~4 MiB window is held, so the frame table
/// for an arbitrarily large core is built in O(1) memory. Offsets are absolute stream positions, so
/// on a well-formed stream the result equals [`parse_frames`] (see the test).
pub fn parse_frames_io<R: Read>(reader: R) -> io::Result<Vec<FrameInfo>> {
    parse_frames_io_chunked(reader, 1 << 22)
}

fn parse_frames_io_chunked<R: Read>(mut reader: R, chunk: usize) -> io::Result<Vec<FrameInfo>> {
    let chunk = chunk.max(8);
    let mut buf: Vec<u8> = Vec::new();
    let mut eof = false;
    let mut base = 0usize; // absolute stream offset of buf[0]
    let mut frames = Vec::new();

    while !eof || !buf.is_empty() {
        if !eof {
            let start = buf.len();
            buf.resize(start + chunk, 0);
            let mut filled = start;
            while filled < buf.len() {
                match reader.read(&mut buf[filled..])? {
                    0 => {
                        eof = true;
                        break;
                    }
                    n => filled += n,
                }
            }
            buf.truncate(filled);
        }

        // Record every complete frame the buffer currently holds (mirrors the emit loop in
        // transform_frames_io_chunked, minus the writes).
        let mut pos = 0usize;
        loop {
            if pos + 2 > buf.len() {
                break;
            }
            if buf[pos..pos + 2] != SYNCWORD {
                match find_sync(&buf, pos) {
                    Some(p) => {
                        pos = p;
                        continue;
                    }
                    None => break,
                }
            }
            if pos + 6 > buf.len() {
                break; // need the header to learn the size
            }
            let mut info = read_frame_header(&buf, pos);
            let size = info.size;
            if size == 0 {
                match find_sync(&buf, pos + 1) {
                    Some(p) => {
                        pos = p;
                        continue;
                    }
                    None => break,
                }
            }
            if pos + size > buf.len() {
                break; // frame not fully buffered yet
            }
            info.offset = base + pos; // absolute (matches parse_frames)
            frames.push(info);
            pos += size;
        }

        // Keep an unfinished trailing frame for the next chunk; at EOF it is incomplete -> dropped.
        let drop = if eof { buf.len() } else { pos };
        base += drop;
        buf.drain(..drop);
    }

    Ok(frames)
}

#[cfg(test)]
mod stream_tests {
    use super::*;
    use std::io::Cursor;

    /// Build a synthetic E-AC-3 syncframe of `size` bytes (even, 4..=4096): valid syncword +
    /// `words_per_syncframe` header so `read_frame_header` reports `size`; body filled with `fill`.
    fn synth_frame(size: usize, fill: u8) -> Vec<u8> {
        assert!(size >= 4 && size % 2 == 0 && size <= 4096);
        let words = (size / 2 - 1) as u16; // 11-bit field
        let mut f = vec![fill; size];
        f[0] = 0x0B;
        f[1] = 0x77;
        f[2] = ((words >> 8) & 0x07) as u8; // strmtyp=0, substreamid=0, words[10:8]
        f[3] = (words & 0xFF) as u8; // words[7:0]
        f
    }

    fn synth_stream() -> Vec<u8> {
        let mut d = Vec::new();
        for &s in &[8usize, 16, 4, 32, 6] {
            d.extend_from_slice(&synth_frame(s, 0xAA));
        }
        d
    }

    #[test]
    fn identity_reproduces_input_across_tiny_chunks() {
        let data = synth_stream();
        for &chunk in &[8usize, 7, 13, 1024] {
            let mut out = Vec::new();
            let n =
                transform_frames_io_chunked(Cursor::new(&data), &mut out, chunk, |_i, _n, f| {
                    f.to_vec()
                })
                .unwrap();
            assert_eq!(out, data, "identity must reproduce input (chunk={chunk})");
            assert_eq!(n, 5, "frame count (chunk={chunk})");
        }
    }

    #[test]
    fn passthrough_matches_inject_stream() {
        let data = synth_stream();
        let mem = inject_stream(&data, |_f, _i| None); // None => copy each frame verbatim
        let mut streamed = Vec::new();
        transform_frames_io_chunked(Cursor::new(&data), &mut streamed, 9, |_i, _n, f| f.to_vec())
            .unwrap();
        assert_eq!(streamed, mem);
        assert_eq!(streamed, data);
    }

    #[test]
    fn resync_gaps_copied_verbatim() {
        // Junk (non-syncword) bytes before, between, and after frames must survive untouched.
        let mut data = vec![0x01, 0x02, 0x03];
        data.extend_from_slice(&synth_frame(8, 0xAA));
        data.extend_from_slice(&[0x10, 0x20]); // gap
        data.extend_from_slice(&synth_frame(6, 0xBB));
        data.push(0x55); // trailing
        let mut out = Vec::new();
        transform_frames_io_chunked(Cursor::new(&data), &mut out, 5, |_i, _n, f| f.to_vec())
            .unwrap();
        assert_eq!(out, data);
    }

    #[test]
    fn growth_matches_in_memory_reference() {
        let mut data = vec![0x09]; // leading gap byte
        data.extend_from_slice(&synth_frame(8, 0xAA));
        data.extend_from_slice(&[0x11, 0x22]); // gap
        data.extend_from_slice(&synth_frame(12, 0xCC));
        let grow = |f: &[u8]| {
            let mut v = f.to_vec();
            v.extend_from_slice(b"\xEE\xEE");
            v
        };
        // In-memory reference over parse_frames.
        let frames = parse_frames(&data);
        let mut want = Vec::new();
        let mut cursor = 0usize;
        for f in &frames {
            if f.offset > cursor {
                want.extend_from_slice(&data[cursor..f.offset]);
            }
            want.extend_from_slice(&grow(&data[f.offset..f.offset + f.size]));
            cursor = f.offset + f.size;
        }
        if cursor < data.len() {
            want.extend_from_slice(&data[cursor..]);
        }
        // Streaming, tiny chunk to force frame/gap splits across reads.
        let mut got = Vec::new();
        transform_frames_io_chunked(Cursor::new(&data), &mut got, 5, |_i, _n, f| grow(f)).unwrap();
        assert_eq!(got, want);
    }

    #[test]
    fn parse_frames_io_matches_parse_frames() {
        // The streamed grid-builder must reproduce parse_frames exactly — same frames, same fields,
        // same absolute offsets — at any chunk size, including chunks that split frames and resync
        // gaps across reads. This is what guarantees `frames[i]` lines up with the frame
        // `transform_frames_io` delivers at index `i` in the streamed `atmos` path.
        let mut data = vec![0x09]; // leading gap byte
        data.extend_from_slice(&synth_frame(8, 0xAA));
        data.extend_from_slice(&[0x11, 0x22]); // mid gap
        data.extend_from_slice(&synth_frame(32, 0xCC));
        data.extend_from_slice(&synth_frame(6, 0xBB));
        let want = parse_frames(&data);
        assert_eq!(want.len(), 3, "fixture should parse to 3 frames");
        for &chunk in &[8usize, 7, 13, 64, 1024] {
            let got = parse_frames_io_chunked(Cursor::new(&data), chunk).unwrap();
            assert_eq!(got, want, "parse_frames_io must match parse_frames (chunk={chunk})");
        }
    }
}
