//! Stage A: render the DAMF object essence down to a 5.1 bed (L R C LFE Ls Rs).
//!
//! Each object is panned by its horizontal azimuth onto the standard 5.1 speaker ring
//! using constant-power pairwise (VBAP-style) panning; height (z) folds to the floor.
//! The LFE bed channel passes straight through. Per-object gains are linearly interpolated
//! between the metadata position keyframes so moving objects track smoothly.
//!
//! This 5.1 render is both the immediate playable downmix *and* the core that the JOC
//! layer (Stage D) will later upmix back into objects, so it is not throwaway work.

use anyhow::{Context, Result, ensure};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::Path;

use crate::damf;

/// 5.1 speaker ring as (azimuth degrees, index into the returned `[L,R,C,Ls,Rs]` gain array).
/// Azimuth is measured from front (+y) toward right (+x): 0=C, +30=R, +110=Rs, −30=L, −110=Ls.
const RING: [(f64, usize); 5] = [
    (-110.0, 3), // Ls
    (-30.0, 0),  // L
    (0.0, 2),    // C
    (30.0, 1),   // R
    (110.0, 4),  // Rs
];

/// Constant-power pairwise pan of an object at room position (x,y) to `[L,R,C,Ls,Rs]` gains.
pub(crate) fn pan_vbap(x: f64, y: f64) -> [f32; 5] {
    let mut g = [0.0f32; 5];
    // Azimuth from +y (front) toward +x (right).
    let az = x.atan2(y).to_degrees();
    let n = RING.len();
    for i in 0..n {
        let (a0, idx0) = RING[i];
        let (a1_raw, idx1) = RING[(i + 1) % n];
        // Last segment wraps across the rear gap (Rs +110° → Ls +250°≡−110°).
        let wrap = i + 1 == n;
        let a1 = if wrap { a1_raw + 360.0 } else { a1_raw };
        let azc = if wrap && az < a0 { az + 360.0 } else { az };
        if azc >= a0 && azc <= a1 {
            let frac = (azc - a0) / (a1 - a0);
            let ang = frac * std::f64::consts::FRAC_PI_2;
            g[idx0] = ang.cos() as f32;
            g[idx1] = ang.sin() as f32;
            return g;
        }
    }
    g[2] = 1.0; // unreachable fallback: center
    g
}

/// A 24-bit big-endian PCM sample → f32 in [-1, 1).
#[inline]
pub(crate) fn s24be(b: &[u8]) -> f32 {
    let v = ((b[0] as i32) << 16) | ((b[1] as i32) << 8) | (b[2] as i32);
    let v = (v << 8) >> 8; // sign-extend 24 → 32
    v as f32 / 8_388_608.0
}

/// One object's panning timeline: essence channel + (samplePos, gains) keyframes.
struct ObjEnv {
    ess_ch: usize,
    keys: Vec<(u64, [f32; 5])>,
    cursor: usize,
}

impl ObjEnv {
    /// Gains at absolute sample index `t`, advancing the cursor (t is monotonically increasing).
    fn gains_at(&mut self, t: u64) -> [f32; 5] {
        while self.cursor + 1 < self.keys.len() && self.keys[self.cursor + 1].0 <= t {
            self.cursor += 1;
        }
        let (p0, g0) = self.keys[self.cursor];
        if self.cursor + 1 < self.keys.len() {
            let (p1, g1) = self.keys[self.cursor + 1];
            let frac = if p1 > p0 {
                ((t - p0) as f32) / ((p1 - p0) as f32)
            } else {
                0.0
            };
            let mut g = [0.0f32; 5];
            for k in 0..5 {
                g[k] = g0[k] + (g1[k] - g0[k]) * frac;
            }
            g
        } else {
            g0
        }
    }
}

pub fn downmix(atmos: &Path, out: &Path, gain_db: f64) -> Result<()> {
    let manifest = damf::Manifest::load(atmos)?;
    let dir = atmos.parent().unwrap_or_else(|| Path::new("."));
    let pres = manifest
        .presentations
        .first()
        .context("manifest has no presentations")?;

    let caf = damf::read_caf_info(&dir.join(&pres.audio))?;
    ensure!(
        caf.bits_per_channel == 24 && !caf.is_float(),
        "expected 24-bit int CAF essence"
    );
    let channels = caf.channels as usize;

    // Element ID → essence channel: bed channels first (manifest order), then objects.
    let mut id_to_ch: HashMap<u32, usize> = HashMap::new();
    let mut ch = 0usize;
    let mut lfe_ch: Option<usize> = None;
    for b in &pres.bed_instances {
        for c in &b.channels {
            if c.channel.eq_ignore_ascii_case("LFE") {
                lfe_ch = Some(ch);
            }
            id_to_ch.insert(c.id, ch);
            ch += 1;
        }
    }
    let bed_count = ch;
    for (i, o) in pres.objects.iter().enumerate() {
        id_to_ch.insert(o.id, bed_count + i);
    }

    // Build per-object gain keyframes from the (diff-encoded) metadata events.
    let meta = damf::Metadata::load(&dir.join(&pres.metadata))?;
    let mut cur: HashMap<u32, ([f64; 3], bool)> = HashMap::new();
    let mut keys: HashMap<u32, Vec<(u64, [f32; 5])>> = HashMap::new();
    for e in &meta.events {
        let Some(id) = e.id else { continue };
        let sp = e.sample_pos.unwrap_or(0);
        let st = cur.entry(id).or_insert(([0.0, 0.0, 0.0], true));
        if let Some(p) = &e.pos {
            if p.len() == 3 {
                st.0 = [p[0], p[1], p[2]];
            }
        }
        if let Some(a) = e.active {
            st.1 = a;
        }
        let g = if st.1 {
            pan_vbap(st.0[0], st.0[1])
        } else {
            [0.0; 5]
        };
        keys.entry(id).or_default().push((sp, g));
    }

    // Objects to render = every element that has a position timeline (i.e. not the LFE bed).
    let mut objects: Vec<ObjEnv> = Vec::new();
    for o in &pres.objects {
        if let Some(k) = keys.get(&o.id) {
            objects.push(ObjEnv {
                ess_ch: id_to_ch[&o.id],
                keys: k.clone(),
                cursor: 0,
            });
        }
    }

    let total_frames = caf.frames();
    let lin = 10f32.powf((gain_db / 20.0) as f32);
    log::info!(
        "downmix: {} objects → 5.1, LFE ch {:?}, {} frames ({:.2}s), gain {:+.1} dB",
        objects.len(),
        lfe_ch,
        total_frames,
        total_frames as f64 / caf.sample_rate,
        gain_db,
    );

    // Open input essence at the audio data offset.
    let mut r = BufReader::new(File::open(dir.join(&pres.audio))?);
    r.seek(SeekFrom::Start(caf.data_offset))?;

    // Output: a 32-bit-float 5.1 WAV file, or — when `out` is "-" — raw f32le interleaved PCM to
    // stdout. The raw mode has no 4 GB WAV size cap, so it handles full-length films; pipe it to
    // e.g. `ffmpeg -f f32le -ar <sr> -ac 6 -i - ...`.
    let to_stdout = out == Path::new("-");
    let mut w: BufWriter<Box<dyn Write>> = BufWriter::new(if to_stdout {
        Box::new(std::io::stdout())
    } else {
        Box::new(File::create(out).with_context(|| format!("creating {}", out.display()))?)
    });
    if !to_stdout {
        write_wav_header(&mut w, 6, caf.sample_rate as u32, total_frames)?;
    }

    let block = 8192usize;
    let mut inbuf = vec![0u8; block * channels * 3];
    let mut outbuf: Vec<u8> = Vec::with_capacity(block * 6 * 4);
    let mut peak = [0f32; 6];
    let mut t: u64 = 0;
    let mut remaining = total_frames;

    while remaining > 0 {
        let this = remaining.min(block as u64) as usize;
        let nbytes = this * channels * 3;
        r.read_exact(&mut inbuf[..nbytes])
            .with_context(|| format!("reading essence at frame {t}"))?;
        outbuf.clear();
        for f in 0..this {
            let base = f * channels * 3;
            let mut o = [0f32; 6];
            // LFE bed passes through.
            if let Some(lc) = lfe_ch {
                o[3] = s24be(&inbuf[base + lc * 3..base + lc * 3 + 3]);
            }
            // Objects panned to the 5 mains.
            for obj in &mut objects {
                let s = s24be(&inbuf[base + obj.ess_ch * 3..base + obj.ess_ch * 3 + 3]);
                if s == 0.0 {
                    obj.gains_at(t); // keep cursor advancing
                    continue;
                }
                let g = obj.gains_at(t);
                o[0] += s * g[0]; // L
                o[1] += s * g[1]; // R
                o[2] += s * g[2]; // C
                o[4] += s * g[3]; // Ls
                o[5] += s * g[4]; // Rs
            }
            // Apply makeup gain to mains (not LFE) and write.
            for k in [0usize, 1, 2, 4, 5] {
                o[k] *= lin;
            }
            for k in 0..6 {
                peak[k] = peak[k].max(o[k].abs());
                outbuf.extend_from_slice(&o[k].to_le_bytes());
            }
            t += 1;
        }
        w.write_all(&outbuf)?;
        remaining -= this as u64;
    }
    w.flush()?;

    let db = |v: f32| {
        if v > 0.0 {
            20.0 * v.log10()
        } else {
            f32::NEG_INFINITY
        }
    };
    log::info!(
        "peak dBFS  L {:.1}  R {:.1}  C {:.1}  LFE {:.1}  Ls {:.1}  Rs {:.1}",
        db(peak[0]),
        db(peak[1]),
        db(peak[2]),
        db(peak[3]),
        db(peak[4]),
        db(peak[5]),
    );
    if peak.iter().any(|&p| p > 1.0) {
        log::warn!(
            "output exceeds 0 dBFS (max {:.2}) — consider --gain-db -3",
            peak.iter().cloned().fold(0.0, f32::max)
        );
    }
    log::info!("wrote {}", out.display());
    Ok(())
}

/// Minimal WAVE_FORMAT_EXTENSIBLE / IEEE-float / 5.1 header (sizes known up front).
fn write_wav_header<W: Write>(
    w: &mut W,
    channels: u16,
    sample_rate: u32,
    frames: u64,
) -> Result<()> {
    let bits = 32u16;
    let block_align = channels * bits / 8;
    let byte_rate = sample_rate * block_align as u32;
    let data_size = frames * block_align as u64;
    ensure!(
        data_size + 60 < u32::MAX as u64,
        "output too large for WAV; stream via pipe instead"
    );

    // RIFF size = "WAVE"(4) + fmt chunk(8+40) + data chunk(8+data_size) = 60 + data_size.
    w.write_all(b"RIFF")?;
    w.write_all(&((60 + data_size) as u32).to_le_bytes())?;
    w.write_all(b"WAVE")?;

    w.write_all(b"fmt ")?;
    w.write_all(&40u32.to_le_bytes())?;
    w.write_all(&0xFFFEu16.to_le_bytes())?; // WAVE_FORMAT_EXTENSIBLE
    w.write_all(&channels.to_le_bytes())?;
    w.write_all(&sample_rate.to_le_bytes())?;
    w.write_all(&byte_rate.to_le_bytes())?;
    w.write_all(&block_align.to_le_bytes())?;
    w.write_all(&bits.to_le_bytes())?;
    w.write_all(&22u16.to_le_bytes())?; // cbSize
    w.write_all(&bits.to_le_bytes())?; // wValidBitsPerSample
    w.write_all(&0x3Fu32.to_le_bytes())?; // 5.1 channel mask: FL FR FC LFE BL BR
    // SubFormat GUID: KSDATAFORMAT_SUBTYPE_IEEE_FLOAT
    w.write_all(&[
        0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B,
        0x71,
    ])?;

    w.write_all(b"data")?;
    w.write_all(&(data_size as u32).to_le_bytes())?;
    Ok(())
}
