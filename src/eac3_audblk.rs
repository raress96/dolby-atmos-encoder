//! Minimal E-AC-3 audio-block **bit-walker**, ported from VoidXH/Cavern's transcoder
//! (`Transcoders/EnhancedAC3Body/*`, non-commercial license) which implements ATSC A/52 / ETSI
//! TS 102 366. We do NOT reconstruct audio — we only track the bit position through each audio
//! block so we can locate (and write) the per-block **skip field**, the place real Dolby streams
//! carry the EMDF (OAMD + JOC). See `eac3.rs` for framing; this only handles the audblk region.
//!
//! Specialized to the exact configuration our ffmpeg core produces (verified): independent E-AC-3,
//! 5.1 (acmod 7 + LFE), 6 blocks, 48 kHz, **no coupling, no SPX, no AHT, no delta-bit-allocation**,
//! frame-level exponent strategy (`expstre=0`) and SNR (`snroffststr=0`), fixed bit-alloc params
//! (`bamode=0`). It bails (returns None) if a frame uses anything outside that subset.

use crate::eac3::FrameInfo;
use crate::emdf::BitReader;

// --------------------------------------------------------------------------- constant tables (A/52)

const SLOWDEC: [i32; 4] = [0x0f, 0x11, 0x13, 0x15];
const FASTDEC: [i32; 4] = [0x3f, 0x53, 0x67, 0x7b];
const SLOWGAIN: [i32; 4] = [0x540, 0x4d8, 0x478, 0x410];
const DBPBTAB: [i32; 4] = [0x000, 0x700, 0x900, 0xb00];
const FLOORTAB: [i32; 8] = [0x2f0, 0x2b0, 0x270, 0x230, 0x1f0, 0x170, 0x0f0, -2048];
const FASTGAIN: [i32; 8] = [0x080, 0x100, 0x180, 0x200, 0x280, 0x300, 0x380, 0x400];

const BNDTAB: [u8; 50] = [
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
    27, 28, 31, 34, 37, 40, 43, 46, 49, 55, 61, 67, 73, 79, 85, 97, 109, 121, 133, 157, 181, 205,
    229, 253,
];

const MASKTAB: [u8; 256] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 28, 28, 29, 29, 29, 30, 30, 30, 31, 31, 31, 32, 32, 32, 33, 33, 33, 34, 34, 34, 35, 35, 35,
    35, 35, 35, 36, 36, 36, 36, 36, 36, 37, 37, 37, 37, 37, 37, 38, 38, 38, 38, 38, 38, 39, 39, 39, 39, 39,
    39, 40, 40, 40, 40, 40, 40, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 42, 42, 42, 42, 42, 42, 42,
    42, 42, 42, 42, 42, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 44, 44, 44, 44, 44, 44, 44, 44, 44,
    44, 44, 44, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45,
    45, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 46, 47,
    47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 48, 48, 48,
    48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 49, 49, 49, 49, 49,
    49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 49, 0, 0, 0,
];

const LATAB: [u8; 256] = [
    0x40, 0x3f, 0x3e, 0x3d, 0x3c, 0x3b, 0x3a, 0x39, 0x38, 0x37, 0x36, 0x35, 0x34, 0x34, 0x33, 0x32,
    0x31, 0x30, 0x2f, 0x2f, 0x2e, 0x2d, 0x2c, 0x2c, 0x2b, 0x2a, 0x29, 0x29, 0x28, 0x27, 0x26, 0x26,
    0x25, 0x24, 0x24, 0x23, 0x23, 0x22, 0x21, 0x21, 0x20, 0x20, 0x1f, 0x1e, 0x1e, 0x1d, 0x1d, 0x1c,
    0x1c, 0x1b, 0x1b, 0x1a, 0x1a, 0x19, 0x19, 0x18, 0x18, 0x17, 0x17, 0x16, 0x16, 0x15, 0x15, 0x15,
    0x14, 0x14, 0x13, 0x13, 0x13, 0x12, 0x12, 0x12, 0x11, 0x11, 0x11, 0x10, 0x10, 0x10, 0x0f, 0x0f,
    0x0f, 0x0e, 0x0e, 0x0e, 0x0d, 0x0d, 0x0d, 0x0d, 0x0c, 0x0c, 0x0c, 0x0c, 0x0b, 0x0b, 0x0b, 0x0b,
    0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x09, 0x09, 0x09, 0x09, 0x09, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x05, 0x05,
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
    0x04, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x02,
    0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
    0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];

const HTH: [[i32; 50]; 3] = [
    [
        0x04d0, 0x04d0, 0x0440, 0x0400, 0x03e0, 0x03c0, 0x03b0, 0x03b0, 0x03a0, 0x03a0, 0x03a0,
        0x03a0, 0x03a0, 0x0390, 0x0390, 0x0390, 0x0380, 0x0380, 0x0370, 0x0370, 0x0360, 0x0360,
        0x0350, 0x0350, 0x0340, 0x0340, 0x0330, 0x0320, 0x0310, 0x0300, 0x02f0, 0x02f0, 0x02f0,
        0x02f0, 0x0300, 0x0310, 0x0340, 0x0390, 0x03e0, 0x0420, 0x0460, 0x0490, 0x04a0, 0x0460,
        0x0440, 0x0440, 0x0520, 0x0800, 0x0840, 0x0840,
    ],
    [
        0x04f0, 0x04f0, 0x0460, 0x0410, 0x03e0, 0x03d0, 0x03c0, 0x03b0, 0x03b0, 0x03a0, 0x03a0,
        0x03a0, 0x03a0, 0x03a0, 0x0390, 0x0390, 0x0390, 0x0380, 0x0380, 0x0380, 0x0370, 0x0370,
        0x0360, 0x0360, 0x0350, 0x0350, 0x0340, 0x0340, 0x0320, 0x0310, 0x0300, 0x02f0, 0x02f0,
        0x02f0, 0x02f0, 0x0300, 0x0320, 0x0350, 0x0390, 0x03e0, 0x0420, 0x0450, 0x04a0, 0x0490,
        0x0460, 0x0440, 0x0480, 0x0630, 0x0840, 0x0840,
    ],
    [
        0x0580, 0x0580, 0x04b0, 0x0450, 0x0420, 0x03f0, 0x03e0, 0x03d0, 0x03c0, 0x03b0, 0x03b0,
        0x03b0, 0x03a0, 0x03a0, 0x03a0, 0x03a0, 0x03a0, 0x03a0, 0x03a0, 0x03a0, 0x0390, 0x0390,
        0x0390, 0x0390, 0x0380, 0x0380, 0x0380, 0x0370, 0x0360, 0x0350, 0x0340, 0x0330, 0x0320,
        0x0310, 0x0300, 0x02f0, 0x02f0, 0x02f0, 0x0300, 0x0310, 0x0330, 0x0350, 0x03c0, 0x0410,
        0x0470, 0x04a0, 0x0460, 0x0440, 0x0450, 0x04e0,
    ],
];

const BAPTAB: [u8; 64] = [
    0, 1, 1, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10,
    10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 15,
    15, 15, 15, 15, 15, 15, 15, 15,
];

/// bits per quantized mantissa, indexed by bap (0..15). bap 1,2,4 are grouped; see count_mantissa.
const BITS_TO_READ: [u32; 16] = [0, 5, 7, 3, 7, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16];
const BAP1_BITS: u32 = 5;
const BAP2_BITS: u32 = 7;
const BAP4_BITS: u32 = 7;

const GROUP_ADD: [i32; 3] = [-1, 2, 8];
const GROUP_DIV: [i32; 3] = [3, 6, 12];

/// frmcplexpstr_tbl[frmchexpstr][block] → exp strategy (0=Reuse,1=D15,2=D25,3=D45).
const FRM_EXP_STR: [[u8; 6]; 32] = [
    [1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 3], [1, 0, 0, 0, 2, 0], [1, 0, 0, 0, 3, 3],
    [2, 0, 0, 2, 0, 0], [2, 0, 0, 2, 0, 3], [2, 0, 0, 3, 2, 0], [2, 0, 0, 3, 3, 3],
    [2, 0, 1, 0, 0, 0], [2, 0, 2, 0, 0, 3], [2, 0, 2, 0, 2, 0], [2, 0, 2, 0, 3, 3],
    [2, 0, 3, 2, 0, 0], [2, 0, 3, 2, 0, 3], [2, 0, 3, 3, 2, 0], [2, 0, 3, 3, 3, 3],
    [3, 1, 0, 0, 0, 0], [3, 1, 0, 0, 0, 3], [3, 2, 0, 0, 2, 0], [3, 2, 0, 0, 3, 3],
    [3, 2, 0, 2, 0, 0], [3, 2, 0, 2, 0, 3], [3, 2, 0, 3, 2, 0], [3, 2, 0, 3, 3, 3],
    [3, 3, 1, 0, 0, 0], [3, 3, 2, 0, 0, 3], [3, 3, 2, 0, 2, 0], [3, 3, 2, 0, 3, 3],
    [3, 3, 3, 2, 0, 0], [3, 3, 3, 2, 0, 3], [3, 3, 3, 3, 2, 0], [3, 3, 3, 3, 3, 3],
];

const NCH: usize = 5; // full-bandwidth channels for acmod 7
const MAXBIN: usize = 256;

// --------------------------------------------------------------------------- frame-level state

/// Audio-frame (audfrm) fields we need, parsed from the bitstream.
struct Audfrm {
    /// Bit offset of the `skipFieldSyntaxEnabled` flag in audfrm (so we can flip it).
    skipfld_bit: usize,
    /// Bit offset where audio block 0 begins.
    block0_bit: usize,
    dithflage: bool,
    blkswe: bool,
    snroffststr: u32,
    frmfgaincode: bool,
    frmcsnroffst: u32,
    frmfsnroffst: u32,
    /// Per-block exponent strategy for the 5 full channels (0=Reuse..3=D45).
    chexpstr: [[u8; NCH]; 6],
    /// Per-block LFE exponent strategy flag.
    lfeexpstr: [bool; 6],
    /// Per-block coupling-strategy-present flag (from audfrm); coupling is never *in use*.
    cplstre: [bool; 6],
}

/// Per-channel running allocation buffers (one set reused for all channels/LFE).
struct Alloc {
    exponents: [i32; MAXBIN],
    psd: [i32; MAXBIN],
    bndpsd: [i32; MAXBIN],
    excite: [i32; MAXBIN],
    mask: [i32; MAXBIN],
    bap: [u8; MAXBIN],
}

impl Alloc {
    fn new() -> Self {
        Self {
            exponents: [0; MAXBIN],
            psd: [0; MAXBIN],
            bndpsd: [0; MAXBIN],
            excite: [0; MAXBIN],
            mask: [0; MAXBIN],
            bap: [0; MAXBIN],
        }
    }
}

fn log_add(a: i32, b: i32) -> i32 {
    let c = a - b;
    let address = ((c.abs() >> 1) as usize).min(255);
    if c >= 0 { a + LATAB[address] as i32 } else { b + LATAB[address] as i32 }
}

fn calc_lowcomp(a: i32, b0: i32, b1: i32, bin: usize) -> i32 {
    if bin < 7 {
        if b0 + 256 == b1 {
            return 384;
        } else if b0 > b1 {
            return (a - 64).max(0);
        }
    } else if bin < 20 {
        if b0 + 256 == b1 {
            return 320;
        } else if b0 > b1 {
            return (a - 64).max(0);
        }
    } else {
        return (a - 128).max(0);
    }
    a
}

/// Ungroup + differentially decode exponents into `al.exponents[exp_off..]`, then compute the
/// integrated PSD (`al.bndpsd`). Mirrors Cavern `UngroupExponents`.
fn ungroup_exponents(
    al: &mut Alloc,
    absexp0: i32,
    grouped: &[i32],
    strat: u8,
    start_mant: usize,
    exp_off: usize,
) {
    let grpsize: usize = if strat != 3 { strat as usize } else { 4 };
    let mut absexp = absexp0;
    let mut end_mant = exp_off;
    al.exponents[0] = absexp;
    for &expacc in grouped {
        absexp += expacc / 25 - 2;
        for _ in 0..grpsize {
            al.exponents[end_mant] = absexp;
            end_mant += 1;
        }
        absexp += expacc % 25 / 5 - 2;
        for _ in 0..grpsize {
            al.exponents[end_mant] = absexp;
            end_mant += 1;
        }
        absexp += expacc % 5 - 2;
        for _ in 0..grpsize {
            al.exponents[end_mant] = absexp;
            end_mant += 1;
        }
    }
    for bin in start_mant..end_mant {
        al.psd[bin] = 3072 - (al.exponents[bin] << 7);
    }
    let mut i = start_mant;
    let mut k = MASKTAB[start_mant] as usize;
    loop {
        let lastbin = (BNDTAB[k] as usize).min(end_mant);
        al.bndpsd[k] = al.psd[i];
        i += 1;
        while i < lastbin {
            al.bndpsd[k] = log_add(al.bndpsd[k], al.psd[i]);
            i += 1;
        }
        k += 1;
        if end_mant <= lastbin {
            break;
        }
    }
}

/// Compute `al.bap[start..end]` via the A/52 bit-allocation algorithm (fixed params, no dba, no
/// leak). Mirrors Cavern `Allocate`. `fscod` selects the hearing-threshold table.
fn allocate(al: &mut Alloc, start: usize, end: usize, fgain: i32, snroffset: i32, fscod: usize) {
    let sdecay = SLOWDEC[2];
    let fdecay = FASTDEC[1];
    let sgain = SLOWGAIN[1];
    let dbknee = DBPBTAB[2];
    let floor = FLOORTAB[7];

    let bndstrt = MASKTAB[start] as usize;
    let bndend = MASKTAB[end - 1] as usize + 1;
    let mut fastleak;
    let mut slowleak;
    let mut begin;
    // bndstrt is always 0 for full-bandwidth/LFE channels in our config.
    let mut lowcomp = calc_lowcomp(0, al.bndpsd[0], al.bndpsd[1], 0);
    al.excite[0] = al.bndpsd[0] - fgain - lowcomp;
    lowcomp = calc_lowcomp(lowcomp, al.bndpsd[1], al.bndpsd[2], 1);
    al.excite[1] = al.bndpsd[1] - fgain - lowcomp;
    begin = 7;
    fastleak = 0;
    slowleak = 0;
    for bin in 2..7 {
        if bndend != 7 || bin != 6 {
            lowcomp = calc_lowcomp(lowcomp, al.bndpsd[bin], al.bndpsd[bin + 1], bin);
        }
        fastleak = al.bndpsd[bin] - fgain;
        slowleak = al.bndpsd[bin] - sgain;
        al.excite[bin] = fastleak - lowcomp;
        if (bndend != 7 || bin != 6) && al.bndpsd[bin] <= al.bndpsd[bin + 1] {
            begin = bin + 1;
            break;
        }
    }
    let bins = bndend.min(22);
    for bin in begin..bins {
        if bndend != 7 || bin != 6 {
            lowcomp = calc_lowcomp(lowcomp, al.bndpsd[bin], al.bndpsd[bin + 1], bin);
        }
        fastleak = (fastleak - fdecay).max(al.bndpsd[bin] - fgain);
        slowleak = (slowleak - sdecay).max(al.bndpsd[bin] - sgain);
        al.excite[bin] = (fastleak - lowcomp).max(slowleak);
    }
    begin = 22;
    for bin in begin..bndend {
        fastleak = (fastleak - fdecay).max(al.bndpsd[bin] - fgain);
        slowleak = (slowleak - sdecay).max(al.bndpsd[bin] - sgain);
        al.excite[bin] = fastleak.max(slowleak);
    }

    for bin in bndstrt..bndend {
        if al.bndpsd[bin] < dbknee {
            al.excite[bin] += (dbknee - al.bndpsd[bin]) >> 2;
        }
        al.mask[bin] = al.excite[bin].max(HTH[fscod][bin]);
    }

    let mut i = start;
    let mut j = MASKTAB[start] as usize;
    loop {
        let lastbin = (BNDTAB[j] as usize).min(end);
        let mut masked = al.mask[j] - snroffset - floor;
        if masked < 0 {
            masked = 0;
        }
        masked = (masked & 0x1fe0) + floor;
        while i < lastbin {
            let address = ((al.psd[i] - masked) >> 5).clamp(0, 63);
            al.bap[i] = BAPTAB[address as usize];
            i += 1;
        }
        j += 1;
        if end <= lastbin {
            break;
        }
    }
    for b in al.bap.iter_mut().take(MAXBIN).skip(i) {
        *b = 0;
    }
}

/// Count mantissa bits for `bap[start..end]`, advancing the grouped-quantizer carry positions
/// (`bap1pos`,`bap2pos`,`bap4pos`) across channels exactly like Cavern's DecodeTransformCoeffs.
fn count_mantissa(
    bap: &[u8],
    start: usize,
    end: usize,
    bap1pos: &mut i64,
    bap2pos: &mut i64,
    bap4pos: &mut i64,
) -> u32 {
    let mut mantissa_bits: u32 = 0;
    let mut bap_reads: i64 = 0; // packs counts of bap1..4 in bytes 0..3
    for &b in bap.iter().take(end).skip(start) {
        if b != 0 && b < 5 {
            bap_reads += 1i64 << ((b as i64 - 1) * 8);
        } else {
            mantissa_bits += BITS_TO_READ[b as usize];
        }
    }
    let n1 = bap_reads & 0xFF;
    let n2 = (bap_reads >> 8) & 0xFF;
    let n3 = (bap_reads >> 16) & 0xFF;
    let n4 = bap_reads >> 24;
    mantissa_bits += (((*bap1pos + n1) / 3) as u32) * BAP1_BITS;
    mantissa_bits += (((*bap2pos + n2) / 3) as u32) * BAP2_BITS;
    mantissa_bits += (n3 as u32) * BITS_TO_READ[3];
    mantissa_bits += (((*bap4pos + n4) / 2) as u32) * BAP4_BITS;
    *bap1pos = (*bap1pos + n1) % 3;
    *bap2pos = (*bap2pos + n2) % 3;
    *bap4pos = (*bap4pos + n4) % 2;
    mantissa_bits
}

// --------------------------------------------------------------------------- audfrm parse

/// Parse the audio-frame header. Returns None if the frame uses features outside our subset.
fn parse_audfrm(frame: &[u8], info: &FrameInfo, bsi_end_bit: usize) -> Option<Audfrm> {
    if info.acmod != 7 || !info.lfe || info.blocks != 6 || info.strmtyp != 0 {
        return None;
    }
    let mut r = BitReader::new(frame);
    r.pos = bsi_end_bit;

    // expstre: blocks==6 → read a bit; we require it to be 0 (frame-level strategy).
    let expstre = r.read_bit();
    if expstre {
        return None;
    }
    let ahte = r.read_bit();
    if ahte {
        return None;
    }
    let snroffststr = r.read(2);
    let transproce = r.read_bit();
    let blkswe = r.read_bit();
    let dithflage = r.read_bit();
    let bamode = r.read_bit();
    let frmfgaincode = r.read_bit();
    let dbaflde = r.read_bit();
    let skipfld_bit = r.pos;
    let _skipflde = r.read_bit();
    let spxattene = r.read_bit();
    if transproce || bamode || dbaflde || spxattene {
        return None; // outside our subset
    }

    // Coupling strategy bits in audfrm (acmod 7 > 1).
    let mut cplstre = [false; 6];
    let mut cplinu = [false; 6];
    cplstre[0] = true;
    cplinu[0] = r.read_bit();
    for b in 1..6 {
        cplstre[b] = r.read_bit();
        if cplstre[b] {
            cplinu[b] = r.read_bit();
        } else {
            cplinu[b] = cplinu[b - 1];
        }
    }
    if cplinu.iter().any(|&c| c) {
        return None; // we don't handle coupling
    }

    // Exponent strategy (expstre==0): no coupling blocks → no frmcplexpstr; per-channel frmchexpstr.
    let mut chexpstr = [[0u8; NCH]; 6];
    for ch in 0..NCH {
        let frmchexpstr = r.read(5) as usize;
        for b in 0..6 {
            chexpstr[b][ch] = FRM_EXP_STR[frmchexpstr][b];
        }
    }
    // LFE exponent strategy: one bit per block.
    let mut lfeexpstr = [false; 6];
    for b in 0..6 {
        lfeexpstr[b] = r.read_bit();
    }

    // Converter exponent strategy (independent && blocks==6 → present, no enable bit).
    for _ in 0..NCH {
        r.read(5);
    }

    // Audio frame SNR offset data (snroffststr==0).
    let (mut frmcsnroffst, mut frmfsnroffst) = (0u32, 0u32);
    if snroffststr == 0 {
        frmcsnroffst = r.read(6);
        frmfsnroffst = r.read(4);
    } else {
        return None;
    }

    // transproce==0, spxattene==0 → nothing more.

    // blkstrtinfoe (blocks!=1 → read enable bit, skip the payload if present).
    if r.read_bit() {
        // nblkstrtbits = (blocks-1) * (4 + log2ceil(words_per_syncframe))
        let words = info.size as u32 / 2;
        let log2 = 32 - (words.saturating_sub(1)).leading_zeros();
        let nbits = (info.blocks as u32 - 1) * (4 + log2);
        for _ in 0..nbits {
            r.read(1);
        }
    }

    // dialnorm/compr in second part? No — that's bsi. After blkstrtinfo, the audblks begin.
    Some(Audfrm {
        skipfld_bit,
        block0_bit: r.pos,
        dithflage,
        blkswe,
        snroffststr,
        frmfgaincode,
        frmcsnroffst,
        frmfsnroffst,
        chexpstr,
        lfeexpstr,
        cplstre,
    })
}

/// Walk one audio block from bit position `pos`, returning the bit offset of its **skip-field
/// point** (right before mantissas, where `skiple` is read) and the bit position after the block.
/// `prev_*` carry exponent/strategy state across blocks. Returns None on anything unexpected.
#[allow(clippy::too_many_arguments)]
fn walk_block(
    frame: &[u8],
    af: &Audfrm,
    block: usize,
    mut pos: usize,
    fscod: usize,
    alloc: &mut [Alloc],
    lfe_alloc: &mut Alloc,
    endmant: &mut [usize; NCH],
    lfe_have_exp: &mut bool,
) -> Option<(usize, usize)> {
    let mut r = BitReader::new(frame);
    r.pos = pos;
    let eac3 = true;

    // blkswe==0 → no blksw bits. dithflage==0 → no dithflag bits.
    if af.blkswe {
        for _ in 0..NCH {
            r.read(1);
        }
    }
    if af.dithflage {
        for _ in 0..NCH {
            r.read(1);
        }
    }
    // dynrng (ReadConditional 8)
    if r.read_bit() {
        r.read(8);
    }
    // ReadSPX(block): block 0 → spxstre forced true (no bit); else read spxstre.
    let spxstre = if block == 0 { true } else { r.read_bit() };
    if spxstre {
        let spxinu = r.read_bit();
        if spxinu {
            return None; // SPX in use → outside subset
        }
    }
    // DecodeCouplingStrategy: cplstre[block] known; cplinu always false → for eac3, only resets
    // (no bits) when cplstre[block] is set, nothing when it isn't.
    let _ = af.cplstre[block];

    // Channel bandwidth code: for channels whose exp strategy != Reuse (and not coupled/spx).
    let mut chbwcod = [0u32; NCH];
    for ch in 0..NCH {
        if af.chexpstr[block][ch] != 0 {
            chbwcod[ch] = r.read(6);
        }
    }
    // endmant per channel (no coupling/spx): (chbwcod+12)*3 + 37, only when strategy != Reuse.
    for ch in 0..NCH {
        if af.chexpstr[block][ch] != 0 {
            endmant[ch] = (chbwcod[ch] as usize + 12) * 3 + 37;
        }
    }

    // Channel exponents (when strategy != Reuse): absexp(4) + nchgrps*7 + gainrng(2), then ungroup.
    for ch in 0..NCH {
        let strat = af.chexpstr[block][ch];
        if strat != 0 {
            let nchgrps = (endmant[ch] as i32 + GROUP_ADD[strat as usize - 1])
                / GROUP_DIV[strat as usize - 1];
            let absexp = r.read(4) as i32;
            let mut grouped = Vec::with_capacity(nchgrps as usize);
            for _ in 0..nchgrps {
                grouped.push(r.read(7) as i32);
            }
            r.read(2); // gainrng
            ungroup_exponents(&mut alloc[ch], absexp, &grouped, strat, 0, 1);
        }
    }
    // LFE exponents (when lfeexpstr[block]): absexp(4) + 2*7, ungroup (D15, nlfegrps=2).
    if af.lfeexpstr[block] {
        let absexp = r.read(4) as i32;
        let g0 = r.read(7) as i32;
        let g1 = r.read(7) as i32;
        ungroup_exponents(lfe_alloc, absexp, &[g0, g1], 1, 0, 1);
        *lfe_have_exp = true;
    }

    // bamode==0 → fixed params (no bits). snroffststr==0 → frame-level snr (no bits).
    // eac3 fgaincode = frmfgaincode && ReadBit(); frmfgaincode==0 → no bit, fgaincod=4.
    let mut fgain_read = false;
    if af.frmfgaincode {
        fgain_read = r.read_bit();
    }
    let fgaincod_default = 4usize;
    if fgain_read {
        return None; // per-block fgain not in our subset
    }
    // convsnroffste (independent): 1 bit, +10 if set.
    if eac3 && r.read_bit() {
        r.read(10);
    }
    // cplinu false → no cplleak. dbaflde==0 && eac3 → delta bit allocation skipped entirely.

    // ---- skip-field point ----
    let skip_point = r.pos;

    // Now count mantissa bits to find the block end. bap1/2/4 carry across channels+LFE.
    let csnroffst = af.frmcsnroffst;
    let fsnroffst = af.frmfsnroffst;
    let mut bap1pos: i64 = 2;
    let mut bap2pos: i64 = 2;
    let mut bap4pos: i64 = 1;
    let fgain = FASTGAIN[fgaincod_default];
    let mut mant_bits: u32 = 0;
    for ch in 0..NCH {
        if csnroffst == 0 && fsnroffst == 0 {
            continue; // bap all zero → no mantissa bits
        }
        let snroffset = ((((csnroffst as i32 - 15) << 4) + fsnroffst as i32) << 2) as i32;
        allocate(&mut alloc[ch], 0, endmant[ch], fgain, snroffset, fscod);
        mant_bits += count_mantissa(&alloc[ch].bap, 0, endmant[ch], &mut bap1pos, &mut bap2pos, &mut bap4pos);
    }
    // LFE mantissas (lfeendmant=7), uses the most recent LFE exponents.
    if csnroffst != 0 || fsnroffst != 0 {
        let snroffset = ((((csnroffst as i32 - 15) << 4) + fsnroffst as i32) << 2) as i32;
        allocate(lfe_alloc, 0, 7, fgain, snroffset, fscod);
        mant_bits += count_mantissa(&lfe_alloc.bap, 0, 7, &mut bap1pos, &mut bap2pos, &mut bap4pos);
    }

    pos = skip_point + mant_bits as usize;
    Some((skip_point, pos))
}

/// Walk all 6 audio blocks of a core frame. Returns the per-block skip-field bit offsets and the
/// bit position after the last block (the start of the aux/errorcheck tail). None if unsupported.
pub fn skip_points(frame: &[u8], info: &FrameInfo, bsi_end_bit: usize) -> Option<(Vec<usize>, usize)> {
    let af = parse_audfrm(frame, info, bsi_end_bit)?;
    let fscod = info.fscod as usize;
    if fscod > 2 {
        return None;
    }
    let mut alloc: Vec<Alloc> = (0..NCH).map(|_| Alloc::new()).collect();
    let mut lfe_alloc = Alloc::new();
    let mut endmant = [0usize; NCH];
    let mut lfe_have_exp = false;
    let mut pos = af.block0_bit;
    let mut points = Vec::with_capacity(6);
    for block in 0..6 {
        let (sp, next) = walk_block(
            frame, &af, block, pos, fscod, &mut alloc, &mut lfe_alloc, &mut endmant, &mut lfe_have_exp,
        )?;
        points.push(sp);
        pos = next;
    }
    Some((points, pos))
}

/// Expose the audfrm's `skipFieldSyntaxEnabled` bit offset (for flipping it to 1).
pub fn skipfld_enable_bit(frame: &[u8], info: &FrameInfo, bsi_end_bit: usize) -> Option<usize> {
    parse_audfrm(frame, info, bsi_end_bit).map(|af| af.skipfld_bit)
}
