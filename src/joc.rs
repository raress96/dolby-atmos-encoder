//! JOC (Joint Object Coding) payload decoder + encoder, ETSI TS 103 420 §6 + Annex A.
//!
//! The decoder is ported from VoidXH/Cavern (JointObjectCoding.cs, non-commercial license) and
//! exists to validate our understanding against real Dolby payloads; the encoder is ours.
//!
//! A JOC payload describes, per object, a mixing matrix over the core downmix channels
//! (5 full-bandwidth channels for joc_dmx_config_idx 0) × parameter bands (grouped QMF subbands).
//! The decoder reconstructs each object as `obj[sb] = Σ_ch M[ch][band(sb)] · core_ch[sb]`.
//! Matrix values are quantized around a center then band-differentially Huffman coded.

use crate::emdf::{BitReader, BitWriter};
use crate::joc_tables::*;

/// One object's decoded JOC data (raw integer codes, before dequantization).
#[derive(Debug, Clone, Default)]
pub struct JocObject {
    pub active: bool,
    pub bands_idx: u8,
    pub bands: usize,
    pub sparse: bool,
    /// 0 = coarse (step 0.2, center 48, 96 codes), 1 = fine (step 0.1, center 96, 192 codes).
    pub quant: u8,
    pub steep_slope: bool,
    pub data_points: usize,
    pub timeslot_offsets: [usize; 2],
    /// Sparse mode: source channel codes, `[dp][band]`.
    pub channels: Vec<Vec<i32>>,
    /// Sparse mode: gain codes for the source channel, `[dp][band]`.
    pub vectors: Vec<Vec<i32>>,
    /// Full mode: differential matrix codes, `[dp][ch][band]`.
    pub matrix: Vec<Vec<Vec<i32>>>,
}

/// A decoded JOC frame (header + info + raw data codes).
#[derive(Debug, Clone)]
pub struct JocFrame {
    pub dmx_config: u8,
    pub channel_count: usize,
    pub object_count: usize,
    pub gain_pow: u8,
    pub gain_frac: u8,
    pub seq: u16,
    pub objects: Vec<JocObject>,
    /// Bits consumed from the payload by the parse.
    pub bits_used: usize,
}

fn huff_decode(table: &[[i32; 2]], r: &mut BitReader) -> Result<i32, String> {
    let mut node: i32 = 0;
    loop {
        let bit = r.read(1) as usize;
        node = table[node as usize][bit];
        if node <= -1 {
            return Ok(!node);
        }
        if node as usize >= table.len() {
            return Err(format!("huffman node {node} out of range"));
        }
        if r.pos > r.end {
            return Err("huffman ran past end of payload".into());
        }
    }
}

fn mtx_table(quant: u8) -> &'static [[i32; 2]] {
    if quant == 1 { &JOC_HUFF_CODE_FINE_GENERIC } else { &JOC_HUFF_CODE_COARSE_GENERIC }
}

fn vec_table(quant: u8) -> &'static [[i32; 2]] {
    if quant == 1 { &JOC_HUFF_CODE_FINE_COEFF_SPARSE } else { &JOC_HUFF_CODE_COARSE_COEFF_SPARSE }
}

fn idx_table(channels: usize) -> &'static [[i32; 2]] {
    if channels == 7 { &JOC_HUFF_CODE_7CH_POS_INDEX_SPARSE } else { &JOC_HUFF_CODE_5CH_POS_INDEX_SPARSE }
}

/// Decode a JOC EMDF payload (id 14). Mirrors Cavern's Decode{Header,Info,Data}.
pub fn decode_joc(payload: &[u8]) -> Result<JocFrame, String> {
    let mut r = BitReader::new(payload);

    // joc_header()
    let dmx_config = r.read(3) as u8;
    if dmx_config > 4 {
        return Err(format!("unsupported joc_dmx_config_idx {dmx_config}"));
    }
    let channel_count = if dmx_config == 0 || dmx_config == 3 { 5 } else { 7 };
    let object_count = r.read(6) as usize + 1;
    let ext_config = r.read(3);
    if ext_config != 0 {
        return Err(format!("unsupported joc_ext_config_idx {ext_config}"));
    }

    // joc_info()
    let gain_pow = r.read(3) as u8;
    let gain_frac = r.read(5) as u8;
    let seq = r.read(10) as u16;
    let mut objects = vec![JocObject::default(); object_count];
    for obj in objects.iter_mut() {
        obj.active = r.read_bit();
        if obj.active {
            obj.bands_idx = r.read(3) as u8;
            obj.bands = JOC_NUM_BANDS[obj.bands_idx as usize] as usize;
            obj.sparse = r.read_bit();
            obj.quant = r.read(1) as u8;
            // joc_data_point_info()
            obj.steep_slope = r.read_bit();
            obj.data_points = r.read(1) as usize + 1;
            if obj.steep_slope {
                for dp in 0..obj.data_points {
                    obj.timeslot_offsets[dp] = r.read(5) as usize + 1;
                }
            }
        }
    }

    // joc_data()
    for obj in objects.iter_mut() {
        if !obj.active {
            continue;
        }
        if obj.sparse {
            let ch_tab = idx_table(channel_count);
            let v_tab = vec_table(obj.quant);
            for _dp in 0..obj.data_points {
                let mut chans = Vec::with_capacity(obj.bands);
                chans.push(r.read(3) as i32);
                for _pb in 1..obj.bands {
                    chans.push(huff_decode(ch_tab, &mut r)?);
                }
                let mut vecs = Vec::with_capacity(obj.bands);
                for _pb in 0..obj.bands {
                    vecs.push(huff_decode(v_tab, &mut r)?);
                }
                obj.channels.push(chans);
                obj.vectors.push(vecs);
            }
        } else {
            let tab = mtx_table(obj.quant);
            for _dp in 0..obj.data_points {
                let mut dp_mtx = Vec::with_capacity(channel_count);
                for _ch in 0..channel_count {
                    let mut row = Vec::with_capacity(obj.bands);
                    for _pb in 0..obj.bands {
                        row.push(huff_decode(tab, &mut r)?);
                    }
                    dp_mtx.push(row);
                }
                obj.matrix.push(dp_mtx);
            }
        }
    }

    if r.pos > r.end {
        return Err(format!("payload overrun: used {} of {} bits", r.pos, r.end));
    }
    Ok(JocFrame {
        dmx_config,
        channel_count,
        object_count,
        gain_pow,
        gain_frac,
        seq,
        objects,
        bits_used: r.pos,
    })
}

// --------------------------------------------------------------------------- encoder

/// Fine quantization (quant=1): step 0.1, codes 0..191 around center 96. Matches the config real
/// Dolby encoders use (12 bands, full matrix, fine quant, 1 data point), observed in jocprobe.
const FINE_CENTER: i32 = 96;
const FINE_RANGE: i32 = 192;
const FINE_STEP: f32 = 0.1;
/// joc_num_bands_idx 5 → 12 parameter bands.
const ENC_BANDS_IDX: u32 = 5;
const ENC_BANDS: usize = 12;

/// Symbol → (code, bit length), derived from a decode tree at startup.
struct HuffEnc {
    codes: Vec<(u32, u8)>,
}

impl HuffEnc {
    fn from_tree(table: &[[i32; 2]]) -> Self {
        fn walk(table: &[[i32; 2]], node: usize, code: u32, len: u8, codes: &mut Vec<(u32, u8)>) {
            for bit in 0..2usize {
                let next = table[node][bit];
                let code = (code << 1) | bit as u32;
                if next <= -1 {
                    let sym = (!next) as usize;
                    if sym >= codes.len() {
                        codes.resize(sym + 1, (0, 0));
                    }
                    codes[sym] = (code, len + 1);
                } else {
                    walk(table, next as usize, code, len + 1, codes);
                }
            }
        }
        let mut codes = Vec::new();
        walk(table, 0, 0, 0, &mut codes);
        Self { codes }
    }

    fn put(&self, w: &mut BitWriter, sym: i32) {
        let (code, len) = self.codes[sym as usize];
        debug_assert!(len > 0, "no huffman code for symbol {sym}");
        w.write(code, len as u32);
    }
}

fn fine_mtx_encoder() -> &'static HuffEnc {
    use std::sync::OnceLock;
    static ENC: OnceLock<HuffEnc> = OnceLock::new();
    ENC.get_or_init(|| HuffEnc::from_tree(&JOC_HUFF_CODE_FINE_GENERIC))
}

/// Quantize a broadband mixing gain to a fine-quant code (0..191).
pub fn quantize_gain(g: f32) -> i32 {
    ((g / FINE_STEP).round() as i32 + FINE_CENTER).clamp(0, FINE_RANGE - 1)
}

/// Dequantize back to the gain the decoder will apply (for self-verification).
pub fn dequantize_gain(q: i32) -> f32 {
    (q - FINE_CENTER) as f32 * FINE_STEP
}

/// Encode one JOC frame. `matrices[obj][ch]` = broadband mixing gain reconstructing dynamic
/// object `obj` from core channel `ch` (order L, R, C, Ls, Rs — joc_dmx_config_idx 0; the LFE is
/// bypassed and must not be included). All parameter bands carry the same broadband value, so the
/// band-differential codes after the first band are zeros (1 bit each in the fine table).
pub fn encode_joc(matrices: &[[f32; 5]], seq: u16) -> Vec<u8> {
    let nobj = matrices.len();
    assert!((1..=64).contains(&nobj), "JOC supports 1..=64 objects");
    let enc = fine_mtx_encoder();
    let mut w = BitWriter::new();
    // joc_header()
    w.write(0, 3); // joc_dmx_config_idx = 0 (5.1 core: L R C Ls Rs + bypassed LFE)
    w.write((nobj - 1) as u32, 6);
    w.write(0, 3); // joc_ext_config_idx = 0
    // joc_info()
    w.write(4, 3); // gain = 1 + (0/32)·2^(4−4) = 1.0 (matches real Dolby streams)
    w.write(0, 5);
    w.write(seq as u32 & 0x3FF, 10); // joc_sequence_counter
    for _ in 0..nobj {
        w.bit(1); // b_joc_obj_present
        w.write(ENC_BANDS_IDX, 3);
        w.bit(0); // b_joc_sparse = 0 → full matrix
        w.write(1, 1); // joc_quant_table_idx = fine
        w.bit(0); // b_joc_slope (steep) = 0
        w.write(0, 1); // joc_num_data_points − 1 = 0
    }
    // joc_data()
    for m in matrices {
        for &g in m.iter() {
            let q = quantize_gain(g);
            let v0 = (q - FINE_CENTER).rem_euclid(FINE_RANGE);
            enc.put(&mut w, v0);
            for _ in 1..ENC_BANDS {
                enc.put(&mut w, 0);
            }
        }
    }
    w.into_bytes()
}

/// Least-squares JOC analysis for one frame: given the downmix gains `d[obj][ch]`
/// (core_ch = Σ_obj d[obj][ch]·s_obj, channel order L R C Ls Rs) and per-object signal powers
/// `p[obj]` (mean square over the frame), return the reconstruction matrices `M[obj][ch]`
/// minimizing E‖s_obj − Σ_ch M[obj][ch]·core_ch‖² for uncorrelated objects:
/// M_obj = p_obj·d_obj·(Dᵀ·diag(p)·D + λI)⁻¹. Silent objects get all-zero rows.
pub fn analyze_frame(d: &[[f32; 5]], p: &[f32]) -> Vec<[f32; 5]> {
    assert_eq!(d.len(), p.len());
    let mut c = [[0f64; 5]; 5];
    for (row, &pw) in d.iter().zip(p) {
        for i in 0..5 {
            for j in 0..5 {
                c[i][j] += pw as f64 * row[i] as f64 * row[j] as f64;
            }
        }
    }
    let trace: f64 = (0..5).map(|i| c[i][i]).sum();
    let ridge = trace * 1e-3 / 5.0 + 1e-12;
    for (i, row) in c.iter_mut().enumerate() {
        row[i] += ridge;
    }
    let cinv = invert5(&c);
    d.iter()
        .zip(p)
        .map(|(row, &pw)| {
            let mut m = [0f32; 5];
            for j in 0..5 {
                let mut v = 0f64;
                for i in 0..5 {
                    v += pw as f64 * row[i] as f64 * cinv[i][j];
                }
                m[j] = (v as f32).clamp(-9.5, 9.5);
            }
            m
        })
        .collect()
}

/// Gauss-Jordan inverse of a (well-conditioned, ridge-regularized) 5×5 matrix.
fn invert5(a: &[[f64; 5]; 5]) -> [[f64; 5]; 5] {
    let mut m = *a;
    let mut inv = [[0f64; 5]; 5];
    for (i, row) in inv.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for col in 0..5 {
        let pivot = (col..5)
            .max_by(|&r1, &r2| m[r1][col].abs().partial_cmp(&m[r2][col].abs()).unwrap())
            .unwrap();
        m.swap(col, pivot);
        inv.swap(col, pivot);
        let pv = m[col][col];
        for j in 0..5 {
            m[col][j] /= pv;
            inv[col][j] /= pv;
        }
        for r in 0..5 {
            if r != col {
                let f = m[r][col];
                for j in 0..5 {
                    m[r][j] -= f * m[col][j];
                    inv[r][j] -= f * inv[col][j];
                }
            }
        }
    }
    inv
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joc_roundtrip() {
        let matrices = vec![
            [1.0f32, 0.0, 0.0, 0.0, 0.0],
            [0.0, -0.5, 0.7, 0.0, 0.2],
            [0.3, 0.3, 0.0, 0.9, -9.5],
        ];
        let payload = encode_joc(&matrices, 123);
        let f = decode_joc(&payload).expect("decode");
        assert_eq!(f.object_count, 3);
        assert_eq!(f.channel_count, 5);
        assert_eq!(f.seq, 123);
        assert!(payload.len() * 8 - f.bits_used < 8, "padding only");
        for (obj, want) in f.objects.iter().zip(&matrices) {
            assert!(obj.active && !obj.sparse && obj.quant == 1);
            assert_eq!(obj.bands, ENC_BANDS);
            // Reverse the band-differential coding: all bands must carry the same broadband value.
            for ch in 0..5 {
                let codes = &obj.matrix[0][ch];
                let mut q = (FINE_CENTER + codes[0]).rem_euclid(FINE_RANGE);
                for &v in &codes[1..] {
                    let q2 = (q + v).rem_euclid(FINE_RANGE);
                    assert_eq!(q2, q);
                    q = q2;
                }
                let got = dequantize_gain(q);
                assert!((got - want[ch]).abs() <= FINE_STEP / 2.0 + 1e-6, "ch{ch}: {got} vs {}", want[ch]);
            }
        }
    }

    #[test]
    fn analysis_recovers_isolated_objects() {
        // Three objects panned to distinct channels with distinct powers: the LS solution must
        // reconstruct each object from its own channel with gain ≈ 1.
        let d = vec![
            [1.0f32, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.7, 0.7],
        ];
        let p = vec![1.0f32, 0.25, 0.5];
        let m = analyze_frame(&d, &p);
        assert!((m[0][0] - 1.0).abs() < 0.01, "{:?}", m[0]);
        assert!((m[1][2] - 1.0).abs() < 0.01, "{:?}", m[1]);
        assert!((m[2][3] - 0.714).abs() < 0.02, "{:?}", m[2]); // 0.7/(0.7²+0.7²)·0.7… ≈ 1/1.4
        // A silent object must not grab channel content.
        let m = analyze_frame(&d, &[1.0, 0.0, 1.0]);
        assert!(m[1].iter().all(|&v| v.abs() < 1e-3), "{:?}", m[1]);
    }
}

/// Bit-scan a whole buffer (e.g. a raw E-AC-3 frame) for an EMDF sync at ANY bit offset.
/// Real Dolby streams carry the EMDF in an audio-block skip field, which is rarely byte-aligned.
/// Returns (bit_offset, byte-realigned copy of the rest of the buffer from that offset).
pub fn find_emdf_anywhere(frame: &[u8]) -> Option<(usize, Vec<u8>)> {
    let total_bits = frame.len() * 8;
    let read_bit = |p: usize| (frame[p >> 3] >> (7 - (p & 7))) & 1;
    'outer: for start in 0..total_bits.saturating_sub(16) {
        let mut v: u16 = 0;
        for i in 0..16 {
            v = (v << 1) | read_bit(start + i) as u16;
        }
        if v != 0x5838 {
            continue;
        }
        // Sanity: the 16-bit length must fit in the remaining buffer.
        let mut len: usize = 0;
        for i in 16..32 {
            if start + i >= total_bits {
                continue 'outer;
            }
            len = (len << 1) | read_bit(start + i) as usize;
        }
        if len == 0 || start + 32 + len * 8 > total_bits {
            continue;
        }
        let mut w = BitWriter::new();
        for p in start..total_bits {
            w.bit(read_bit(p) as u32);
        }
        return Some((start, w.into_bytes()));
    }
    None
}
