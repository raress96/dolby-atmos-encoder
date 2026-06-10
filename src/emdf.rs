//! EMDF container + Object Audio Metadata (OAMD) encoder/decoder.
//!
//! Bit layout follows ETSI TS 103 420 §5.5 (OAMD) and ETSI TS 102 366 Annex H.2 (the EMDF
//! container), cross-checked against VoidXH/Cavern's decoder. The EMDF carries the OAMD (and later
//! JOC) payloads in the E-AC-3 aux field; see `eac3::inject_aux`. The Atmos *flag* itself
//! (flag_ec3_extension_type_a + complexity_index) lives separately in `addbsi` — not here.
//!
//! All bit I/O is MSB-first (matches the E-AC-3 / Cavern convention).

use std::sync::OnceLock;

/// EMDF marker (byte-aligned syncword).
const EMDF_SYNC: u16 = 0x5838;
/// EMDF payload id for Object Audio Metadata.
pub const PAYLOAD_OAMD: u8 = 11;
/// EMDF payload id for Joint Object Coding.
pub const PAYLOAD_JOC: u8 = 14;

// --------------------------------------------------------------------------- bit I/O

/// MSB-first bit writer.
pub(crate) struct BitWriter {
    out: Vec<u8>,
    cur: u8,
    nbits: u8,
}

impl BitWriter {
    pub(crate) fn new() -> Self {
        Self { out: Vec::new(), cur: 0, nbits: 0 }
    }

    #[inline]
    pub(crate) fn bit(&mut self, b: u32) {
        self.cur = (self.cur << 1) | (b & 1) as u8;
        self.nbits += 1;
        if self.nbits == 8 {
            self.out.push(self.cur);
            self.cur = 0;
            self.nbits = 0;
        }
    }

    /// Write the low `n` bits of `val`, MSB-first.
    pub(crate) fn write(&mut self, val: u32, n: u32) {
        for i in (0..n).rev() {
            self.bit((val >> i) & 1);
        }
    }

    /// Copy `nbits` MSB-first bits from `data`.
    pub(crate) fn copy_bits(&mut self, data: &[u8], nbits: usize) {
        for i in 0..nbits {
            self.bit(((data[i >> 3] >> (7 - (i & 7))) & 1) as u32);
        }
    }

    /// `variable_bits` (ETSI TS 103 420 §5.5.1 with a large group cap): chunked base-2^n.
    /// Only emits up to 3 groups — sufficient for every value we produce (< 2^12); panics otherwise.
    pub(crate) fn write_var(&mut self, value: u32, n: u32) {
        let base = 1u32 << n;
        if value < base {
            self.write(value, n);
            self.bit(0);
            return;
        }
        let off2 = base;
        if value <= off2 + base * base - 1 {
            let v = value - off2;
            self.write(v / base, n);
            self.bit(1);
            self.write(v % base, n);
            self.bit(0);
            return;
        }
        let off3 = base + base * base;
        if value <= off3 + base * base * base - 1 {
            let v = value - off3;
            self.write(v / (base * base), n);
            self.bit(1);
            self.write((v / base) % base, n);
            self.bit(1);
            self.write(v % base, n);
            self.bit(0);
            return;
        }
        panic!("write_var: value {value} too large for n={n} (>3 groups)");
    }

    pub(crate) fn bit_len(&self) -> usize {
        self.out.len() * 8 + self.nbits as usize
    }

    /// Byte-padded snapshot of the bits written so far, plus the exact bit count.
    pub(crate) fn snapshot(&self) -> (Vec<u8>, usize) {
        let nbits = self.bit_len();
        let mut bytes = self.out.clone();
        if self.nbits != 0 {
            bytes.push(self.cur << (8 - self.nbits));
        }
        (bytes, nbits)
    }

    /// Flush to bytes, zero-padding the final partial byte (MSB-aligned).
    pub(crate) fn into_bytes(mut self) -> Vec<u8> {
        if self.nbits != 0 {
            self.cur <<= 8 - self.nbits;
            self.out.push(self.cur);
        }
        self.out
    }
}

/// MSB-first bit reader.
pub(crate) struct BitReader<'a> {
    data: &'a [u8],
    pub(crate) pos: usize,
    pub(crate) end: usize,
}

impl<'a> BitReader<'a> {
    pub(crate) fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0, end: data.len() * 8 }
    }

    pub(crate) fn read(&mut self, n: u32) -> u32 {
        let mut v = 0;
        for _ in 0..n {
            let bit = if self.pos < self.end {
                (self.data[self.pos >> 3] >> (7 - (self.pos & 7))) & 1
            } else {
                0
            };
            v = (v << 1) | bit as u32;
            self.pos += 1;
        }
        v
    }

    pub(crate) fn read_bit(&mut self) -> bool {
        self.read(1) == 1
    }

    /// `variable_bits_max(n, max_groups)` per ETSI TS 103 420 §5.5.1.
    pub(crate) fn read_var(&mut self, n: u32, max_groups: u32) -> u32 {
        let mut value = self.read(n);
        let mut more = self.read_bit();
        let mut num_group = 1u32;
        if max_groups > num_group {
            if more {
                value = (value << n) + (1 << n);
            }
            while more {
                value += self.read(n);
                more = self.read_bit();
                if num_group >= max_groups {
                    break;
                }
                if more {
                    value = (value << n) + (1 << n);
                    num_group += 1;
                }
            }
        }
        value
    }
}

// --------------------------------------------------------------------------- OAMD

/// One dynamic object's state at a frame: position in DAMF coordinates
/// (x: −1 left → +1 right, y: −1 back → +1 front, z: 0 floor → 1 ceiling).
#[derive(Debug, Clone, Copy)]
pub struct ObjectPos {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Quantize DAMF coordinates to OAMD position codes.
/// OAMD: X 0=left..1=right (×1/62), Y 0=front..1=back (×1/62), Z 0=floor..1=ceiling (×1/15).
fn quantize_pos(p: ObjectPos) -> (u32, u32, u32, u32) {
    let oamd_x = ((p.x + 1.0) * 0.5).clamp(0.0, 1.0);
    let oamd_y = ((1.0 - p.y) * 0.5).clamp(0.0, 1.0);
    let oamd_z = p.z.clamp(0.0, 1.0);
    let px = (oamd_x * 62.0).round() as u32; // 0..62
    let py = (oamd_y * 62.0).round() as u32;
    let pz = (oamd_z * 15.0).round() as u32; // 0..15
    (px, py, 1, pz) // z sign always positive (z >= 0)
}

/// Encode an OAMD payload for a "dynamic object-only + LFE bed" program:
/// object 0 is the LFE bed, objects 1.. are the dynamic objects. Single info block at frame start.
/// Returns whole bytes (the EMDF payload body).
pub fn encode_oamd(objects: &[ObjectPos], lfe: bool) -> Vec<u8> {
    let bed_count: usize = if lfe { 1 } else { 0 };
    let object_count = bed_count + objects.len();
    assert!(object_count >= 1 && object_count <= 159, "object_count out of range");

    let mut w = BitWriter::new();
    // object_audio_metadata_payload()
    w.write(0, 2); // oa_md_version_bits
    write_count5(&mut w, (object_count - 1) as u32); // object_count_bits (=count-1)
    // program_assignment(): dynamic-object-only + LFE
    w.bit(1); // b_dyn_object_only_program
    w.bit(if lfe { 1 } else { 0 }); // b_lfe_present
    w.bit(0); // b_alternate_object_data_present
    w.write(1, 4); // oa_element_count_bits = 1 element

    // oa_element_md()
    w.write(1, 4); // oa_element_id_idx = 1 (object element)
    // oa_element_size_bits = variable_bits_max(4,4) = (element body bits) - 1.
    let body = encode_object_element(objects, bed_count, object_count);
    let size_value = (body.bit_len - 1) as u32;
    write_var_max4(&mut w, size_value);
    // (b_alternate_object_data_present == 0 → no alternate id)
    w.bit(0); // b_discard_unknown_element
    w.copy_bits(&body.bytes, body.bit_len); // oa_element() = object_element()

    w.into_bytes()
}

struct Bits {
    bytes: Vec<u8>,
    bit_len: usize,
}

fn encode_object_element(objects: &[ObjectPos], bed_count: usize, object_count: usize) -> Bits {
    let mut w = BitWriter::new();
    // md_update_info()
    w.write(0, 2); // sample_offset_code = 0 (no offset)
    w.write(0, 3); // num_obj_info_blocks_bits = 0 → 1 block
    // block_update_info(0)
    w.write(0, 6); // block_offset_factor_bits = 0 (start of frame)
    w.write(0b10, 2); // ramp_duration_code = 2 → 1536-sample ramp (smooth, full frame)

    w.bit(1); // b_reserved_data_not_present = 1 (no reserved 5 bits)

    // object_data() for each object (single block, blk == 0). Objects [0, bed_count) are beds.
    for obj in 0..object_count {
        let is_bed = obj < bed_count;
        let pos = if is_bed { None } else { objects.get(obj - bed_count).copied() };
        encode_object_info_block(&mut w, is_bed, pos);
    }

    let bit_len = w.bit_len();
    Bits { bytes: w.into_bytes(), bit_len }
}

fn encode_object_info_block(w: &mut BitWriter, is_bed: bool, pos: Option<ObjectPos>) {
    w.bit(0); // b_object_not_active = 0 (active)
    // blk == 0 ⇒ object_basic_info_status_idx = 0b01 (implicit) ⇒ object_basic_info()
    //   object_basic_info[] = {true, true}
    // object_gain_idx: real Dolby files use 0b00 (unity) for the LFE bed but 0b11 ("reuse last
    // gain" → no per-frame attenuation) for dynamic objects. Match that exactly.
    w.write(if is_bed { 0b00 } else { 0b11 }, 2);
    w.bit(1); // b_default_object_priority = 1 (no priority bits)

    if !is_bed {
        // blk == 0, not bed ⇒ object_render_info_status_idx = 0b01 ⇒ object_render_info()
        //   obj_render_info[] = {true, true, true, true}
        let p = pos.unwrap_or(ObjectPos { x: 0.0, y: 0.0, z: 0.0 });
        let (px, py, zsign, pz) = quantize_pos(p);
        // obj_render_info[0]: position (blk==0 ⇒ no differential bit)
        w.write(px, 6); // pos3D_X_bits
        w.write(py, 6); // pos3D_Y_bits
        w.write(zsign, 1); // pos3D_Z_sign_bits
        w.write(pz, 4); // pos3D_Z_bits
        w.bit(0); // b_object_distance_specified = 0
        // obj_render_info[1]: zone_constraints_idx(3) + b_enable_elevation(1). Every real Dolby file
        // sets this field to 0b0001 (elevation ENABLED); we previously wrote 0b0000, which Cavern
        // ignores but a conformant Dolby renderer may read as "height disabled" → flattened objects.
        w.write(0b0001, 4);
        // obj_render_info[2]: object_size_idx = 0b00 (point source)
        w.write(0b00, 2);
        // obj_render_info[3]: b_object_use_screen_ref = 0
        w.bit(0);
        // b_object_snap = 0
        w.bit(0);
    }

    w.bit(0); // b_additional_table_data_exists = 0
}

/// object_count_bits: 5 bits, with a 7-bit extension when == 0x1F.
fn write_count5(w: &mut BitWriter, count_minus_1: u32) {
    if count_minus_1 < 0x1F {
        w.write(count_minus_1, 5);
    } else {
        w.write(0x1F, 5);
        w.write(count_minus_1 - 0x1F, 7);
    }
}

/// variable_bits_max(4, 4) for oa_element_size_bits.
fn write_var_max4(w: &mut BitWriter, value: u32) {
    w.write_var(value, 4);
}

// --------------------------------------------------------------------------- EMDF protection seam
//
// `emdf_protection` (ETSI TS 102 366 v1.4.1 Annex H.2.1.4 / H.2.2) is a KEYED authentication code,
// not a plain checksum. The spec states `key_id` selects an "authentication key" and that the
// calculation of `protection_bits_primary` / `protection_bits_secondary` is "implementation
// dependent and is not defined in the present document." No open decoder (ffmpeg, Cavern, truehdd)
// computes or validates it; Dolby-certified hardware does. We proved empirically that no public CRC
// reproduces Dolby's values (see `verify_emdf_protection` + the brute-force documented in README).
//
// This trait is the ONE seam where a real signer plugs in. The default `PublicCrcProtector` keeps
// the historical behaviour (well-formed but unsigned). A `KeyedProtector` signs with a supplied key
// via `dolby_keyed_mac` — which today is a structurally-valid stand-in, because Dolby's actual
// construction is undocumented. The day someone supplies BOTH a valid key AND that construction,
// only `dolby_keyed_mac` changes; the rest of the pipeline already carries the field end-to-end.

/// Protection-length selector → field width in bits. Index 0 ("reserved" for primary) treated as 0.
pub const PROT_LEN: [u32; 4] = [0, 8, 32, 128];

/// The seam where the (proprietary, keyed) `emdf_protection` computation lives.
pub trait EmdfProtector: Send + Sync {
    /// 3-bit `key_id` written into the EMDF header — which authentication key signs this stream.
    fn key_id(&self) -> u8 {
        0
    }
    /// (primary, secondary) 2-bit length selectors. Real Dolby DD+ JOC uses (2, 1) = 32-bit primary
    /// + 8-bit secondary, which we mirror by default.
    fn length_codes(&self) -> (u8, u8) {
        (2, 1)
    }
    /// Primary protection value over `covered` = first `nbits` body bits (emdf_version .. end of the
    /// 2+2 length-selector bits), MSB-first byte-padded.
    fn primary(&self, covered: &[u8], nbits: usize) -> u128;
    /// Secondary protection value over `covered` = the same region PLUS the primary bits.
    fn secondary(&self, covered: &[u8], nbits: usize) -> u128;
}

/// Default protector: public, deterministic CRCs — NOT Dolby's keyed MAC. The emitted field is
/// well-formed and round-trips through every software oracle that ignores protection (ffmpeg,
/// Cavern), but it is *unsigned*: Dolby-certified hardware that validates the real signature rejects
/// it. This is the honest default — structurally valid, cryptographically absent.
pub struct PublicCrcProtector;

impl EmdfProtector for PublicCrcProtector {
    fn primary(&self, covered: &[u8], nbits: usize) -> u128 {
        crc32_bits(covered, nbits) as u128
    }
    fn secondary(&self, covered: &[u8], nbits: usize) -> u128 {
        crc8_bits(covered, nbits) as u128
    }
}

/// Keyed protector: signs the EMDF via `dolby_keyed_mac` using a supplied key. Selected from the CLI
/// (`--emdf-key` / `DOLBY_EMDF_KEY`). See `dolby_keyed_mac` for the (undocumented) construction caveat.
pub struct KeyedProtector {
    pub key: Vec<u8>,
    pub key_id: u8,
}

impl EmdfProtector for KeyedProtector {
    fn key_id(&self) -> u8 {
        self.key_id
    }
    fn primary(&self, covered: &[u8], nbits: usize) -> u128 {
        dolby_keyed_mac(&self.key, self.key_id, covered, nbits, 32)
    }
    fn secondary(&self, covered: &[u8], nbits: usize) -> u128 {
        dolby_keyed_mac(&self.key, self.key_id, covered, nbits, 8)
    }
}

/// THE proprietary seam. Dolby's EMDF protection is a keyed MAC whose algorithm is not public
/// (ETSI TS 102 366 H.2.2.4: "implementation dependent and is not defined"). This stand-in computes
/// HMAC-SHA256 over `key_id || covered_bytes` and truncates to the top `out_bits` bits — a real
/// keyed MAC structurally, but it will NOT match Dolby's signature unless/until the exact
/// construction (covered region, MAC, byte/bit ordering, truncation, per-`key_id` handling) is
/// implemented here. Replace the body of this function — and only this function — to make signed
/// output valid on certified hardware.
fn dolby_keyed_mac(key: &[u8], key_id: u8, covered: &[u8], nbits: usize, out_bits: u32) -> u128 {
    use hmac::{Hmac, Mac};
    use sha2::Sha256;
    let mut mac = <Hmac<Sha256>>::new_from_slice(key).expect("HMAC accepts any key length");
    mac.update(&[key_id]);
    let nbytes = nbits.div_ceil(8).min(covered.len());
    mac.update(&covered[..nbytes]);
    let tag = mac.finalize().into_bytes();
    let mut v: u128 = 0;
    for i in 0..out_bits as usize {
        v = (v << 1) | ((tag[i >> 3] >> (7 - (i & 7))) & 1) as u128;
    }
    v
}

/// Active protector (process-global). Set once from the CLI; defaults to the public-CRC placeholder.
static PROTECTOR: OnceLock<Box<dyn EmdfProtector>> = OnceLock::new();
static FALLBACK_PROTECTOR: PublicCrcProtector = PublicCrcProtector;

/// Install the active EMDF protector. Call once, early in `main`. No-op if already set.
pub fn set_protector(p: Box<dyn EmdfProtector>) {
    let _ = PROTECTOR.set(p);
}

fn active_protector() -> &'static dyn EmdfProtector {
    match PROTECTOR.get() {
        Some(b) => b.as_ref(),
        None => &FALLBACK_PROTECTOR,
    }
}

/// Append a protection value of `n` bits (MSB-first) to the body writer.
fn write_protection_bits(b: &mut BitWriter, val: u128, n: u32) {
    for i in (0..n).rev() {
        b.bit(((val >> i) & 1) as u32);
    }
}

// --------------------------------------------------------------------------- EMDF container

/// Wrap one or more (payload_id, payload_bytes) into an EMDF container, with the payload-config
/// values mandated by ETSI TS 103 420 §8.2 Table 56 (duratione=0, groupide=1, codecdatae=0,
/// discard_unknown=0, frame-aligned, priority/proc_allowed=0). Returns whole bytes starting 0x58 0x38.
pub fn wrap_emdf(payloads: &[(u8, Vec<u8>)]) -> Vec<u8> {
    let prot = active_protector();
    // Body after the 16-bit length field.
    let mut b = BitWriter::new();
    b.write(0, 2); // emdf_version = 0
    b.write(prot.key_id() as u32, 3); // key_id (selects the authentication key)
    for (id, payload) in payloads {
        b.write(*id as u32, 5); // emdf_payload_id
        b.bit(0); // smploffste = 0
        b.bit(0); // duratione = 0
        b.bit(1); // groupide = 1
        b.write_var(0, 2); // groupid = 0
        b.bit(0); // codecdatae = 0 (matches real Dolby JOC streams)
        b.bit(0); // discard_unknown_payload = 0 → enter config block
        b.bit(1); // payload_frame_aligned = 1 (smploffste == 0)
        b.write(0, 2); // create_duplicate = 0, remove_duplicate = 0
        b.write(0, 7); // priority + proc_allowed = 0
        b.write_var(payload.len() as u32, 8); // emdf_payload_size (bytes)
        b.copy_bits(payload, payload.len() * 8);
    }
    b.write(0, 5); // emdf_payload_id = 0 → terminate
    // emdf_protection() — ETSI TS 102 366 H.2.1.4. Field widths and the signature itself come from
    // the active EmdfProtector: the public-CRC placeholder by default, or a keyed signer when a key
    // is supplied (--emdf-key / DOLBY_EMDF_KEY). See the "EMDF protection seam" above for why a
    // conformant signature is a proprietary keyed MAC we cannot reproduce open-source.
    let (pc, sc) = prot.length_codes();
    b.write(pc as u32, 2); // protection_length_primary
    b.write(sc as u32, 2); // protection_length_secondary
    let (snap, nbits) = b.snapshot();
    let prim = prot.primary(&snap, nbits);
    write_protection_bits(&mut b, prim, PROT_LEN[pc as usize]); // protection_bits_primary
    let (snap, nbits) = b.snapshot();
    let sec = prot.secondary(&snap, nbits);
    write_protection_bits(&mut b, sec, PROT_LEN[sc as usize]); // protection_bits_secondary
    let body = b.into_bytes();

    let mut w = BitWriter::new();
    w.write(EMDF_SYNC as u32, 16);
    w.write(body.len() as u32, 16); // emdf length in bytes (after this field)
    w.copy_bits(&body, body.len() * 8);
    w.into_bytes()
}

/// CRC-32 (poly 0x04C11DB7, init 0, MSB-first) over the first `nbits` bits of `data`.
fn crc32_bits(data: &[u8], nbits: usize) -> u32 {
    let mut crc: u32 = 0;
    for i in 0..nbits {
        let b = ((data[i >> 3] >> (7 - (i & 7))) & 1) as u32;
        let top = (crc >> 31) & 1;
        crc <<= 1;
        if b ^ top != 0 {
            crc ^= 0x04C1_1DB7;
        }
    }
    crc
}

/// CRC-8 (poly 0x2F, init 0, MSB-first) over the first `nbits` bits of `data`. Used for the EMDF
/// protection field — deterministic and well-formed; the spec leaves the exact algorithm undefined.
fn crc8_bits(data: &[u8], nbits: usize) -> u8 {
    let mut crc: u8 = 0;
    for i in 0..nbits {
        let b = (data[i >> 3] >> (7 - (i & 7))) & 1;
        let top = (crc >> 7) & 1;
        crc <<= 1;
        if b ^ top != 0 {
            crc ^= 0x2F;
        }
    }
    crc
}

/// Convenience: build the full EMDF (OAMD only, for now) for one frame.
pub fn encode_frame_emdf(objects: &[ObjectPos], lfe: bool) -> Vec<u8> {
    let oamd = encode_oamd(objects, lfe);
    wrap_emdf(&[(PAYLOAD_OAMD, oamd)])
}

// --------------------------------------------------------------------------- decode (round-trip)

/// A decoded object position (OAMD raw codes + DAMF-space reconstruction).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DecodedObject {
    pub px: u32,
    pub py: u32,
    pub pz: u32,
    pub is_bed: bool,
}

/// Decoded OAMD summary, for round-trip validation. Mirrors Cavern's decoder closely enough to
/// recover object count and per-object positions for the single-block program we emit.
#[derive(Debug, Clone)]
pub struct DecodedOamd {
    pub object_count: usize,
    pub bed_count: usize,
    pub objects: Vec<DecodedObject>,
}

/// Find the EMDF sync (byte-aligned 0x5838) in an aux byte buffer and decode the OAMD payload.
pub fn decode_emdf_oamd(aux: &[u8]) -> Option<DecodedOamd> {
    // Byte-scan for the syncword.
    let mut start = None;
    for i in 0..aux.len().saturating_sub(1) {
        if aux[i] == 0x58 && aux[i + 1] == 0x38 {
            start = Some(i);
            break;
        }
    }
    let start = start?;
    let mut r = BitReader::new(&aux[start..]);
    let _sync = r.read(16);
    let _len = r.read(16);
    let _version = r.read(2);
    let _key = r.read(3);
    loop {
        let id = r.read(5) as u8;
        if id == 0 {
            return None; // no OAMD found
        }
        let _smploffste = r.read_bit();
        if _smploffste {
            r.read(12); // sample_offset
        }
        if r.read_bit() {
            r.read_var(11, 8);
        } // duratione
        if r.read_bit() {
            r.read_var(2, 8);
        } // groupide
        if r.read_bit() {
            r.read(8);
        } // codecdatae
        if !r.read_bit() {
            // discard_unknown_payload == 0
            let mut frame_aligned = false;
            if !_smploffste {
                frame_aligned = r.read_bit();
                if frame_aligned {
                    r.read(2);
                }
            }
            if _smploffste || frame_aligned {
                r.read(7);
            }
        }
        let size = r.read_var(8, 8) as usize; // bytes
        let payload_end = r.pos + size * 8;
        if id == PAYLOAD_OAMD {
            return Some(decode_oamd(&mut r));
        }
        r.pos = payload_end;
    }
}

/// List every EMDF payload in an aux byte buffer as (payload_id, size_bytes). For analysis of
/// real streams (e.g. is JOC=14 present? is OAMD=11 present? what sizes?).
pub fn list_emdf_payloads(aux: &[u8]) -> Option<(usize, Vec<(u8, usize)>)> {
    let mut start = None;
    for i in 0..aux.len().saturating_sub(1) {
        if aux[i] == 0x58 && aux[i + 1] == 0x38 {
            start = Some(i);
            break;
        }
    }
    let start = start?;
    let mut r = BitReader::new(&aux[start..]);
    let _sync = r.read(16);
    let emdf_len = r.read(16) as usize;
    let _version = r.read(2);
    let _key = r.read(3);
    let mut out = Vec::new();
    loop {
        if r.pos + 5 > r.end {
            break;
        }
        let id = r.read(5) as u8;
        if id == 0 {
            break;
        }
        let smploffste = r.read_bit();
        if smploffste {
            r.read(12);
        }
        if r.read_bit() {
            r.read_var(11, 8);
        }
        if r.read_bit() {
            r.read_var(2, 8);
        }
        if r.read_bit() {
            r.read(8);
        }
        if !r.read_bit() {
            let mut fa = false;
            if !smploffste {
                fa = r.read_bit();
                if fa {
                    r.read(2);
                }
            }
            if smploffste || fa {
                r.read(7);
            }
        }
        let size = r.read_var(8, 8) as usize;
        out.push((id, size));
        r.pos += size * 8;
        if r.pos > r.end {
            break;
        }
    }
    Some((emdf_len, out))
}

/// Like `list_emdf_payloads`, but also returns each payload's bytes (re-aligned to byte 0).
/// `buf` must start at (or before) a byte-aligned EMDF sync; for non-byte-aligned EMDF
/// (real Dolby skip-field carriage), pre-shift the buffer (see `joc::find_emdf_anywhere`).
pub fn extract_emdf_payloads(buf: &[u8]) -> Option<Vec<(u8, Vec<u8>)>> {
    let mut start = None;
    for i in 0..buf.len().saturating_sub(1) {
        if buf[i] == 0x58 && buf[i + 1] == 0x38 {
            start = Some(i);
            break;
        }
    }
    let start = start?;
    let mut r = BitReader::new(&buf[start..]);
    let _sync = r.read(16);
    let _emdf_len = r.read(16) as usize;
    let _version = r.read(2);
    let _key = r.read(3);
    let mut out = Vec::new();
    loop {
        if r.pos + 5 > r.end {
            break;
        }
        let id = r.read(5) as u8;
        if id == 0 {
            break;
        }
        let smploffste = r.read_bit();
        if smploffste {
            r.read(12);
        }
        if r.read_bit() {
            r.read_var(11, 8);
        }
        if r.read_bit() {
            r.read_var(2, 8);
        }
        if r.read_bit() {
            r.read(8);
        }
        if !r.read_bit() {
            let mut fa = false;
            if !smploffste {
                fa = r.read_bit();
                if fa {
                    r.read(2);
                }
            }
            if smploffste || fa {
                r.read(7);
            }
        }
        let size = r.read_var(8, 8) as usize;
        if r.pos + size * 8 > r.end {
            break;
        }
        // Re-align the payload to bit 0 of a fresh buffer.
        let mut w = BitWriter::new();
        for i in 0..size * 8 {
            let p = r.pos + i;
            w.bit(((buf[start + (p >> 3)] >> (7 - (p & 7))) & 1) as u32);
        }
        out.push((id, w.into_bytes()));
        r.pos += size * 8;
    }
    Some(out)
}

fn decode_oamd(r: &mut BitReader) -> DecodedOamd {
    let _ver = r.read(2);
    let object_count = read_count5(r) as usize + 1;
    // program_assignment
    let bed_count = if r.read_bit() {
        // dynamic-object-only program
        if r.read_bit() { 1 } else { 0 } // b_lfe_present
    } else {
        0 // (other program types unused by our encoder)
    };
    let _alt = r.read_bit();
    let element_count = r.read(4);

    let mut objects = Vec::new();
    for _ in 0..element_count {
        let _elem_id = r.read(4);
        let size = r.read_var(4, 4) as usize; // element size bits - 1
        let elem_end = r.pos + size + 1;
        let _discard = r.read_bit();
        // object_element → md_update_info
        let _soc = r.read(2);
        let num_blocks = r.read(3) as usize + 1;
        for _ in 0..num_blocks {
            let _bof = r.read(6);
            let rdc = r.read(2);
            if rdc == 3 {
                if r.read_bit() {
                    r.read(4);
                } else {
                    r.read(11);
                }
            }
        }
        let _resv = r.read_bit();
        // object_data: object_count objects × num_blocks blocks
        for obj in 0..object_count {
            let is_bed = obj < bed_count;
            for blk in 0..num_blocks {
                if let Some(d) = decode_object_info_block(r, blk, is_bed) {
                    objects.push(d);
                }
            }
        }
        r.pos = elem_end;
    }

    DecodedOamd { object_count, bed_count, objects }
}

fn decode_object_info_block(r: &mut BitReader, blk: usize, is_bed: bool) -> Option<DecodedObject> {
    let not_active = r.read_bit();
    let basic_status = if not_active {
        0
    } else if blk == 0 {
        1
    } else {
        r.read(2)
    };
    if basic_status == 1 || basic_status == 3 {
        // object_basic_info
        let info = if basic_status == 1 { 0b11 } else { r.read(2) };
        if info & 0b10 != 0 {
            let gain_idx = r.read(2);
            if gain_idx == 0b10 {
                r.read(6);
            }
        }
        if info & 0b01 != 0 && !r.read_bit() {
            r.read(5);
        }
    }
    let render_status = if not_active {
        0
    } else if !is_bed {
        if blk == 0 { 1 } else { r.read(2) }
    } else {
        0
    };
    let mut decoded = None;
    if render_status == 1 || render_status == 3 {
        let info = if render_status == 1 { 0b1111 } else { r.read(4) };
        if info & 1 != 0 {
            let differential = blk != 0 && r.read_bit();
            let (px, py, pz);
            if differential {
                r.read(3);
                r.read(3);
                r.read(3);
                px = 0;
                py = 0;
                pz = 0;
            } else {
                px = r.read(6);
                py = r.read(6);
                let _sign = r.read(1);
                pz = r.read(4);
            }
            if r.read_bit() && !r.read_bit() {
                r.read(4);
            }
            decoded = Some(DecodedObject { px, py, pz, is_bed });
        }
        if info & 2 != 0 {
            r.read(4);
        }
        if info & 4 != 0 {
            match r.read(2) {
                1 => {
                    r.read(5);
                }
                2 => {
                    r.read(15);
                }
                _ => {}
            }
        }
        if info & 8 != 0 && r.read_bit() {
            r.read(3);
            r.read(2);
        }
        r.read(1); // b_object_snap
    }
    if r.read_bit() {
        // b_additional_table_data_exists
        let n = r.read(4) + 1;
        r.read(n * 8);
    }
    decoded.or(if is_bed { Some(DecodedObject { px: 0, py: 0, pz: 0, is_bed: true }) } else { None })
}

/// Verbose field-by-field dump of a raw OAMD payload (starting at oa_md_version). Mirrors
/// `decode_oamd` / `decode_object_info_block` exactly but prints every field, for diffing our
/// encoder against real Dolby payloads. Reports whether the parse lands cleanly at the element end.
pub fn dump_oamd_verbose(payload: &[u8]) {
    let mut r = BitReader::new(payload);
    let ver = r.read(2);
    let object_count = read_count5(&mut r) as usize + 1;
    let b_dyn = r.read_bit();
    let (bed_count, lfe) = if b_dyn {
        let lfe = r.read_bit();
        (if lfe { 1usize } else { 0 }, lfe)
    } else {
        (0, false)
    };
    let alt = r.read_bit();
    let element_count = r.read(4);
    println!(
        "OAMD ({} B): version={ver} object_count={object_count} dyn_only={} lfe={} alt={} elements={element_count}",
        payload.len(), b_dyn as u8, lfe as u8, alt as u8
    );
    for e in 0..element_count {
        let elem_id = r.read(4);
        let size = r.read_var(4, 4) as usize;
        let elem_end = r.pos + size + 1;
        let discard = r.read_bit();
        let soc = r.read(2);
        let num_blocks = r.read(3) as usize + 1;
        println!(
            "  elem[{e}]: id={elem_id} size_bits={} discard={} sample_offset_code={soc} num_blocks={num_blocks}",
            size + 1, discard as u8
        );
        for blk in 0..num_blocks {
            let bof = r.read(6);
            let rdc = r.read(2);
            let mut extra = String::new();
            if rdc == 3 {
                if r.read_bit() {
                    extra = format!(" ramp4={}", r.read(4));
                } else {
                    extra = format!(" ramp11={}", r.read(11));
                }
            }
            println!("    block[{blk}]: offset_factor={bof} ramp_code={rdc}{extra}");
        }
        let resv = r.read_bit();
        println!("    b_reserved_data_not_present={}", resv as u8);
        for obj in 0..object_count {
            let is_bed = obj < bed_count;
            for blk in 0..num_blocks {
                dump_object_info_block(&mut r, blk, is_bed, obj);
            }
        }
        let used_end = r.pos;
        let payload_bits = payload.len() * 8;
        let pad = elem_end.saturating_sub(used_end);
        println!(
            "    [parse used to bit {used_end}; elem_end={elem_end} (pad {pad}); payload_bits={payload_bits}] {}",
            if used_end == elem_end { "CLEAN" } else { "MISALIGNED vs our model" }
        );
        r.pos = elem_end;
    }
}

fn dump_object_info_block(r: &mut BitReader, blk: usize, is_bed: bool, obj: usize) {
    let tag = if is_bed { " (bed)" } else { "" };
    let not_active = r.read_bit();
    if not_active {
        if r.read_bit() {
            let n = r.read(4) + 1;
            r.read(n * 8);
        }
        println!("    obj {obj:2}{tag}: active=0");
        return;
    }
    let basic_status = if blk == 0 { 1 } else { r.read(2) };
    let mut gain_s = "n/a".to_string();
    let mut prio_s = "n/a".to_string();
    if basic_status == 1 || basic_status == 3 {
        let info = if basic_status == 1 { 0b11 } else { r.read(2) };
        if info & 0b10 != 0 {
            let g = r.read(2);
            gain_s = if g == 0b10 { format!("{g}(g6={})", r.read(6)) } else { format!("{g}") };
        }
        if info & 0b01 != 0 {
            prio_s = if r.read_bit() { "default".into() } else { format!("{}", r.read(5)) };
        }
    }
    let render_status = if !is_bed {
        if blk == 0 { 1 } else { r.read(2) }
    } else {
        0
    };
    let mut render_s = "none".to_string();
    if render_status == 1 || render_status == 3 {
        let info = if render_status == 1 { 0b1111 } else { r.read(4) };
        let mut parts: Vec<String> = Vec::new();
        if info & 1 != 0 {
            let differential = blk != 0 && r.read_bit();
            if differential {
                let (dx, dy, dz) = (r.read(3), r.read(3), r.read(3));
                parts.push(format!("posΔ dx={dx} dy={dy} dz={dz}"));
            } else {
                let (px, py, sign, pz) = (r.read(6), r.read(6), r.read(1), r.read(4));
                let x = px as f32 / 62.0 * 2.0 - 1.0;
                let y = 1.0 - py as f32 / 62.0 * 2.0;
                let z = pz as f32 / 15.0;
                parts.push(format!("pos px={px} py={py} zsign={sign} pz={pz} (x={x:.2} y={y:.2} z={z:.2})"));
            }
            if r.read_bit() {
                parts.push(if r.read_bit() { "dist=inf".into() } else { format!("dist={}", r.read(4)) });
            } else {
                parts.push("dist=0".into());
            }
        }
        if info & 2 != 0 {
            parts.push(format!("zone+elev={:04b}", r.read(4)));
        }
        if info & 4 != 0 {
            let s = r.read(2);
            parts.push(match s {
                1 => format!("size_idx=1 sz={}", r.read(5)),
                2 => format!("size_idx=2 sz3d={}", r.read(15)),
                _ => format!("size_idx={s}"),
            });
        }
        if info & 8 != 0 {
            if r.read_bit() {
                parts.push(format!("screen=1 {},{}", r.read(3), r.read(2)));
            } else {
                parts.push("screen=0".into());
            }
        }
        parts.push(format!("snap={}", r.read(1)));
        render_s = parts.join(" ");
    }
    let mut add_s = String::new();
    if r.read_bit() {
        let n = r.read(4) + 1;
        r.read(n * 8);
        add_s = format!(" +addtbl({n}B)");
    }
    println!("    obj {obj:2}{tag}: active=1 gain_idx={gain_s} prio={prio_s} render[{render_s}]{add_s}");
}

/// Check whether OUR emdf_protection CRC algorithm reproduces the protection bits embedded in a
/// real EMDF. `buf` must start at (or before) a byte-aligned EMDF sync. The decisive question:
/// does our crc32/crc8 == the stream's stored protection? If not, every container we emit carries
/// a protection field a conformant Dolby decoder would reject as corrupt.
pub fn verify_emdf_protection(buf: &[u8]) -> Option<String> {
    let mut start = None;
    for i in 0..buf.len().saturating_sub(1) {
        if buf[i] == 0x58 && buf[i + 1] == 0x38 {
            start = Some(i);
            break;
        }
    }
    let start = start?;
    let bit = |p: usize| -> u32 { ((buf[start + (p >> 3)] >> (7 - (p & 7))) & 1) as u32 };
    let mut r = BitReader::new(&buf[start..]);
    let _sync = r.read(16);
    let _emdf_len = r.read(16);
    let body_start = r.pos; // 32: body begins at emdf_version
    let _version = r.read(2);
    let _key = r.read(3);
    loop {
        if r.pos + 5 > r.end {
            return Some("ran off end before payload terminator".into());
        }
        let id = r.read(5) as u8;
        if id == 0 {
            break;
        }
        let smploffste = r.read_bit();
        if smploffste {
            r.read(12);
        }
        if r.read_bit() {
            r.read_var(11, 8);
        }
        if r.read_bit() {
            r.read_var(2, 8);
        }
        if r.read_bit() {
            r.read(8);
        }
        if !r.read_bit() {
            let mut fa = false;
            if !smploffste {
                fa = r.read_bit();
                if fa {
                    r.read(2);
                }
            }
            if smploffste || fa {
                r.read(7);
            }
        }
        let size = r.read_var(8, 8) as usize;
        r.pos += size * 8;
        if r.pos > r.end {
            return Some("payload overran buffer".into());
        }
    }
    let lenmap = [0u32, 8, 32, 128];
    let prim_bits = lenmap[r.read(2) as usize];
    let sec_bits = lenmap[r.read(2) as usize];
    let protection_start = r.pos;
    // OUR primary CRC32 covers body bits [version .. end of protection-length fields].
    let n_prim = protection_start - body_start;
    let mut wp = BitWriter::new();
    for i in 0..n_prim {
        wp.bit(bit(body_start + i));
    }
    let (snap_p, _) = wp.snapshot();
    let our_prim = crc32_bits(&snap_p, n_prim);
    let stored_prim = if prim_bits <= 32 { r.read(prim_bits) } else { r.read(32) };
    // OUR secondary CRC8 covers body bits [version .. end of primary].
    let n_sec = protection_start + prim_bits as usize - body_start;
    let mut ws = BitWriter::new();
    for i in 0..n_sec {
        ws.bit(bit(body_start + i));
    }
    let (snap_s, _) = ws.snapshot();
    let our_sec = crc8_bits(&snap_s, n_sec) as u32;
    let stored_sec = if sec_bits <= 8 { r.read(sec_bits) } else { r.read(8) };
    // Machine-readable line for offline brute-forcing: emit the ENTIRE EMDF container (from sync,
    // MSB-first byte-packed) plus the bit positions, so any sub-region can be tried as the CRC input.
    let total_bits = protection_start + prim_bits as usize + sec_bits as usize;
    let nbytes = total_bits.div_ceil(8);
    let hex_full: String = buf[start..start + nbytes.min(buf.len() - start)]
        .iter().map(|b| format!("{b:02x}")).collect();
    Some(format!(
        "prot_len: primary={prim_bits} bits, secondary={sec_bits} bits\n  \
         primary  : stored=0x{stored_prim:08x}  ours=0x{our_prim:08x}  {}\n  \
         secondary: stored=0x{stored_sec:02x}        ours=0x{our_sec:02x}        {}\n  \
         FULL body_start=32 prot_start={protection_start} prim_bits={prim_bits} sec_bits={sec_bits} \
         prim=0x{stored_prim:08x} sec=0x{stored_sec:02x} hex={hex_full}",
        if our_prim == stored_prim { "MATCH ✓" } else { "MISMATCH ✗ — our CRC algo ≠ Dolby's" },
        if our_sec == stored_sec { "MATCH ✓" } else { "MISMATCH ✗ — our CRC algo ≠ Dolby's" },
    ))
}

fn read_count5(r: &mut BitReader) -> u32 {
    let v = r.read(5);
    if v == 0x1F { v + r.read(7) } else { v }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn variable_bits_roundtrip() {
        for &(v, n) in &[(0u32, 4), (15, 4), (16, 4), (271, 4), (272, 4), (407, 4), (4367, 4),
            (0, 8), (255, 8), (3, 2)] {
            let mut w = BitWriter::new();
            w.write_var(v, n);
            let total = w.bit_len();
            let bytes = w.into_bytes();
            let mut r = BitReader::new(&bytes);
            let got = r.read_var(n, 4.max(8));
            assert_eq!(got, v, "var roundtrip n={n}");
            assert!(r.pos <= total + 7);
        }
    }

    #[test]
    fn oamd_emdf_roundtrip() {
        // 13 dynamic objects scattered around + 1 LFE bed.
        let objs: Vec<ObjectPos> = (0..13)
            .map(|i| {
                let t = i as f32 / 12.0;
                ObjectPos { x: t * 2.0 - 1.0, y: 1.0 - t * 2.0, z: t }
            })
            .collect();
        let emdf = encode_frame_emdf(&objs, true);
        assert_eq!(&emdf[0..2], &[0x58, 0x38], "EMDF sync");

        let decoded = decode_emdf_oamd(&emdf).expect("decode OAMD");
        assert_eq!(decoded.object_count, 14, "1 LFE + 13 objects");
        assert_eq!(decoded.bed_count, 1);
        let dyn_objs: Vec<_> = decoded.objects.iter().filter(|o| !o.is_bed).collect();
        assert_eq!(dyn_objs.len(), 13, "13 dynamic objects recovered");

        // Positions should round-trip through quantization.
        for (i, o) in dyn_objs.iter().enumerate() {
            let (px, py, _s, pz) = quantize_pos(objs[i]);
            assert_eq!((o.px, o.py, o.pz), (px, py, pz), "object {i} position");
        }
    }
}
