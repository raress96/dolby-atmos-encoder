//! damf2joc — convert a Dolby Atmos Master (DAMF, as produced by `truehdd decode`)
//! into E-AC-3 with Joint Object Coding (Dolby Digital Plus + Atmos).
//!
//! This is an in-progress reimplementation of the relevant part of Dolby's Encoding
//! Engine. See ../convert-poc/DESIGN.md for the architecture and staged plan.

mod damf;
mod eac3;
mod eac3_audblk;
mod emdf;
mod joc;
mod joc_tables;
mod render;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

#[derive(Parser)]
#[command(
    name = "damf2joc",
    version,
    about = "Convert a Dolby Atmos Master (DAMF) to E-AC-3 JOC (DD+ Atmos)"
)]
struct Cli {
    /// Hex Dolby EMDF authentication key (or `@path` to a file of hex). When set, the encoder signs
    /// the EMDF protection field with the keyed protector instead of the public-CRC placeholder.
    /// NOTE: Dolby's keyed-MAC construction is undocumented (see `emdf::dolby_keyed_mac`) — a key
    /// alone does NOT make output hardware-valid.
    #[arg(long, global = true, env = "DOLBY_EMDF_KEY")]
    emdf_key: Option<String>,
    /// key_id (0..7) written alongside a supplied `--emdf-key`.
    #[arg(long, global = true, default_value_t = 0)]
    emdf_key_id: u8,
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Inspect a DAMF master: report bed/object layout, audio essence, and metadata summary.
    Inspect {
        /// Path to the `.atmos` manifest file.
        atmos: PathBuf,
    },
    /// Render the DAMF objects to a 5.1 bed WAV (L R C LFE Ls Rs), 32-bit float.
    Downmix {
        /// Path to the `.atmos` manifest file.
        atmos: PathBuf,
        /// Output WAV path.
        #[arg(short, long, default_value = "downmix.wav")]
        out: PathBuf,
        /// Makeup gain applied to the 5 main channels, in dB.
        #[arg(long, default_value_t = 0.0)]
        gain_db: f64,
    },
    /// Parse a raw E-AC-3 (.eac3/.ec3) elementary stream and report its syncframe structure.
    Eac3probe {
        /// Path to the raw E-AC-3 elementary stream.
        input: PathBuf,
    },
    /// Inject a dummy aux-data payload into every frame (grows frames, fixes crc2)
    /// and self-verify the round-trip. Validates the non-destructive injection path
    /// before real OAMD/JOC payloads go in.
    Eac3inject {
        /// Path to the raw E-AC-3 elementary stream.
        input: PathBuf,
        /// Output E-AC-3 path.
        #[arg(short, long, default_value = "injected.eac3")]
        out: PathBuf,
        /// Hex marker payload to inject into each frame (e.g. DEADBEEF).
        #[arg(long, default_value = "DEADBEEF")]
        marker: String,
    },
    /// Build per-frame OAMD (object positions) from a DAMF master and inject it as EMDF into a
    /// 5.1 E-AC-3 core, producing an object-metadata-bearing stream. (JOC side-info comes later.)
    Oamd {
        /// 5.1 E-AC-3 core (as produced by ffmpeg from `downmix`).
        core: PathBuf,
        /// Path to the `.atmos` manifest file.
        atmos: PathBuf,
        /// Output E-AC-3 path.
        #[arg(short, long, default_value = "oamd.eac3")]
        out: PathBuf,
    },
    /// Full Atmos signalling: inject BOTH the addbsi detection flag (flag_ec3_extension_type_a +
    /// complexity_index) AND the per-frame OAMD EMDF payload. This is the stream to mux + test on
    /// hardware. (JOC object reconstruction is still TODO — heights may not render yet.)
    Atmos {
        /// 5.1 E-AC-3 core (as produced by ffmpeg from `downmix`).
        core: PathBuf,
        /// Path to the `.atmos` manifest file.
        atmos: PathBuf,
        /// Output E-AC-3 path.
        #[arg(short, long, default_value = "atmos.eac3")]
        out: PathBuf,
    },
    /// Find + decode the JOC payload in a real DD+ Atmos stream (EMDF at any bit offset, e.g.
    /// skip-field carriage). Dumps one frame in detail, then aggregate stats over all frames.
    Jocprobe {
        /// Path to the raw E-AC-3 elementary stream.
        input: PathBuf,
        /// Frame index to dump in detail.
        #[arg(long, default_value_t = 0)]
        frame: usize,
    },
    /// Validate the audio-block bit-walker: walk every frame's 6 blocks and check the walk lands
    /// exactly at the frame's aux/errorcheck tail (self-consistency for skip-field carriage).
    Walkprobe {
        /// Path to the raw E-AC-3 elementary stream (the 5.1 core).
        input: PathBuf,
    },
    /// Bisection test: graft a REFERENCE DD+ Atmos stream's exact frame-0 EMDF bytes + addbsi into
    /// every frame of my core. Isolates "my metadata encoder" from "my core+carriage+addbsi".
    Graft {
        /// My 5.1 E-AC-3 core (ffmpeg output) to graft into.
        core: PathBuf,
        /// Reference real DD+ Atmos stream to copy EMDF + addbsi from.
        reference: PathBuf,
        /// Output E-AC-3 path.
        #[arg(short, long, default_value = "graft.eac3")]
        out: PathBuf,
    },
    /// Dump every bsi() field of frame 0 (for diffing my ffmpeg core vs a real Dolby core).
    Bsidump {
        /// Path to the raw E-AC-3 elementary stream.
        input: PathBuf,
    },
    /// Verbose field-by-field dump of a raw OAMD payload given as hex (for diffing our OAMD encoder
    /// against real Dolby payloads, e.g. the bytes printed by `eac3probe`).
    Oamddump {
        /// OAMD payload bytes as hex (e.g. "1f 84 4b 80 ...").
        hex: String,
    },
    /// Check whether OUR emdf_protection CRC reproduces the protection bits of a real Dolby EMDF.
    Emdfverify {
        /// Path to a raw E-AC-3 stream that carries real Dolby EMDF (e.g. working_joc.eac3).
        input: PathBuf,
    },
    /// Definitive test: splice MY EMDF into a REAL Dolby core's skip field in place (coupled audio
    /// untouched). If the result plays Atmos on HW, the core encoder was the wall.
    Coregraft {
        /// Real Dolby DD+ Atmos core to splice into (provides the coupled audio + native skip field).
        realcore: PathBuf,
        /// My injected stream to take per-frame EMDF (OAMD+JOC) from.
        myatmos: PathBuf,
        /// Output E-AC-3 path.
        #[arg(short, long, default_value = "coregraft.eac3")]
        out: PathBuf,
        /// Override addbsi complexity (default: keep the real core's value). Use 14 for my metadata.
        #[arg(long)]
        complexity: Option<u8>,
    },
}

/// Parse a `--emdf-key` value: raw hex, or `@path` to a file of hex (whitespace / `0x` ignored).
fn parse_emdf_key(s: &str) -> Result<Vec<u8>> {
    let text = match s.strip_prefix('@') {
        Some(path) => {
            std::fs::read_to_string(path).with_context(|| format!("reading key file {path}"))?
        }
        None => s.to_string(),
    };
    let hex: String = text.chars().filter(|c| c.is_ascii_hexdigit()).collect();
    anyhow::ensure!(!hex.is_empty() && hex.len() % 2 == 0, "key must be non-empty, even-length hex");
    (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).map_err(Into::into))
        .collect()
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let cli = Cli::parse();
    if let Some(k) = cli.emdf_key.as_deref() {
        let key = parse_emdf_key(k)?;
        log::warn!(
            "EMDF: signing with KEYED protector (key_id={}, {} key bytes). Dolby's keyed-MAC \
             construction is undocumented — output is hardware-valid ONLY if emdf::dolby_keyed_mac \
             implements the real algorithm; otherwise this is a structurally-valid stand-in.",
            cli.emdf_key_id,
            key.len()
        );
        emdf::set_protector(Box::new(emdf::KeyedProtector { key, key_id: cli.emdf_key_id }));
    }
    match cli.command {
        Command::Inspect { atmos } => inspect(&atmos),
        Command::Downmix { atmos, out, gain_db } => render::downmix(&atmos, &out, gain_db),
        Command::Eac3probe { input } => eac3probe(&input),
        Command::Eac3inject { input, out, marker } => eac3inject(&input, &out, &marker),
        Command::Oamd { core, atmos, out } => oamd_inject(&core, &atmos, &out),
        Command::Atmos { core, atmos, out } => atmos_inject(&core, &atmos, &out),
        Command::Jocprobe { input, frame } => jocprobe(&input, frame),
        Command::Walkprobe { input } => walkprobe(&input),
        Command::Graft { core, reference, out } => graft(&core, &reference, &out),
        Command::Coregraft { realcore, myatmos, out, complexity } => coregraft(&realcore, &myatmos, &out, complexity),
        Command::Bsidump { input } => {
            let data = std::fs::read(&input)?;
            let frames = eac3::parse_frames(&data);
            for (i, f) in frames.iter().take(3).enumerate() {
                println!("frame {i}:");
                eac3::bsi_dump(&data[f.offset..f.offset + f.size], f);
            }
            Ok(())
        }
        Command::Oamddump { hex } => {
            let bytes = hex_bytes(&hex).context("parsing OAMD hex")?;
            emdf::dump_oamd_verbose(&bytes);
            Ok(())
        }
        Command::Emdfverify { input } => {
            let data = std::fs::read(&input)?;
            let frames = eac3::parse_frames(&data);
            anyhow::ensure!(!frames.is_empty(), "no E-AC-3 syncframes in {}", input.display());
            let mut checked = 0usize;
            for f in &frames {
                let frame = &data[f.offset..f.offset + f.size];
                if let Some((_, aligned)) = joc::find_emdf_anywhere(frame) {
                    if let Some(report) = emdf::verify_emdf_protection(&aligned) {
                        println!("frame off {}: {report}", f.offset);
                        checked += 1;
                        if checked >= 3 {
                            break;
                        }
                    }
                }
            }
            anyhow::ensure!(checked > 0, "no EMDF found to verify");
            Ok(())
        }
    }
}

/// Validate the audio-block walker: the walk of all 6 blocks must end exactly at the frame's
/// aux/errorcheck tail (frame_bits − 18 for our auxdatae=0 core), proving the mantissa bit-counts
/// (and thus the bit allocation) are correct. Also dumps the per-block skip-field bit offsets.
fn walkprobe(input: &Path) -> Result<()> {
    let data = std::fs::read(input).with_context(|| format!("reading {}", input.display()))?;
    let frames = eac3::parse_frames(&data);
    anyhow::ensure!(!frames.is_empty(), "no E-AC-3 syncframes found");

    let (mut ok, mut unsupported, mut mismatch) = (0usize, 0usize, 0usize);
    let mut first_detail = true;
    for (i, f) in frames.iter().enumerate() {
        let frame = &data[f.offset..f.offset + f.size];
        let Some(bsi_end) = eac3::audfrm_start_bit(frame, f) else {
            unsupported += 1;
            continue;
        };
        let Some((points, end)) = eac3_audblk::skip_points(frame, f, bsi_end) else {
            unsupported += 1;
            continue;
        };
        // Expected tail: crc2(16) + crcrsv(1) + auxdatae(1) = 18 bits (auxdatae=0 in our core).
        let frame_bits = f.size * 8;
        let expected = frame_bits as i64 - 18;
        let diff = expected - end as i64;
        if diff == 0 {
            ok += 1;
        } else {
            mismatch += 1;
        }
        if (first_detail || diff != 0) && i < 3 {
            println!(
                "frame {i}: blocks end at bit {end}, expected tail at {expected} (diff {diff}); frame={frame_bits}b",
            );
            println!("  skip points: {points:?}");
            first_detail = false;
        }
    }
    println!(
        "\nwalk OK (lands at tail) {ok}/{} | mismatch {mismatch} | unsupported {unsupported}",
        frames.len()
    );
    Ok(())
}

/// Decode JOC from a real stream: per-frame EMDF bit-scan → payload 14 → joc::decode_joc.
fn jocprobe(input: &Path, detail_frame: usize) -> Result<()> {
    let data = std::fs::read(input).with_context(|| format!("reading {}", input.display()))?;
    let frames = eac3::parse_frames(&data);
    anyhow::ensure!(!frames.is_empty(), "no E-AC-3 syncframes found");
    println!("{} frames", frames.len());

    let mut emdf_found = 0usize;
    let mut joc_ok = 0usize;
    let mut joc_err: Vec<(usize, String)> = Vec::new();
    let mut seqs: Vec<u16> = Vec::new();

    for (idx, f) in frames.iter().enumerate() {
        let frame = &data[f.offset..f.offset + f.size];
        let Some((bit_off, aligned)) = joc::find_emdf_anywhere(frame) else {
            continue;
        };
        emdf_found += 1;
        let Some(payloads) = emdf::extract_emdf_payloads(&aligned) else {
            continue;
        };
        let joc_payload = payloads.iter().find(|(id, _)| *id == emdf::PAYLOAD_JOC);
        let detail = idx == detail_frame;
        if detail {
            println!("\nframe {idx}: EMDF at bit {bit_off} of {}", f.size * 8);
            for (id, p) in &payloads {
                println!("  payload id={id:2} size={} B", p.len());
            }
        }
        let Some((_, payload)) = joc_payload else {
            continue;
        };
        match joc::decode_joc(payload) {
            Ok(jf) => {
                joc_ok += 1;
                seqs.push(jf.seq);
                if detail {
                    println!(
                        "\n  JOC: dmx_config={} ({}ch core), {} objects, gain={}*2^({}-4)/32+1, seq={}",
                        jf.dmx_config, jf.channel_count, jf.object_count, jf.gain_frac, jf.gain_pow, jf.seq
                    );
                    println!("  bits used {} / {} ({} B payload)", jf.bits_used, payload.len() * 8, payload.len());
                    for (o, obj) in jf.objects.iter().enumerate() {
                        if !obj.active {
                            println!("  obj {o:2}: inactive");
                            continue;
                        }
                        println!(
                            "  obj {o:2}: bands={:2} sparse={} quant={} steep={} dp={} offs={:?}",
                            obj.bands, obj.sparse as u8, obj.quant, obj.steep_slope as u8,
                            obj.data_points, &obj.timeslot_offsets[..obj.data_points]
                        );
                        if obj.sparse {
                            for dp in 0..obj.data_points {
                                println!("    dp{dp} ch  {:?}", obj.channels[dp]);
                                println!("    dp{dp} vec {:?}", obj.vectors[dp]);
                            }
                        } else {
                            for dp in 0..obj.data_points {
                                for (ch, row) in obj.matrix[dp].iter().enumerate() {
                                    println!("    dp{dp} ch{ch} {:?}", row);
                                }
                            }
                        }
                    }
                }
            }
            Err(e) => {
                if joc_err.len() < 5 {
                    joc_err.push((idx, e));
                }
            }
        }
    }

    println!("\nEMDF found in {emdf_found}/{} frames; JOC decoded OK in {joc_ok}", frames.len());
    if !seqs.is_empty() {
        let monotonic = seqs.windows(2).all(|w| w[1] == (w[0] + 1) % 1024);
        println!("seq counter: first={} last={} monotonic(mod 1024)={}", seqs[0], seqs[seqs.len() - 1], monotonic);
    }
    for (idx, e) in &joc_err {
        println!("JOC decode error at frame {idx}: {e}");
    }
    Ok(())
}

/// Shared setup for the OAMD/Atmos injectors: object timelines + the parsed core frames.
struct AtmosCtx {
    timelines: Vec<PosTimeline>,
    nobj: usize,
    data: Vec<u8>,
    frames: Vec<eac3::FrameInfo>,
    start_samples: Vec<u64>,
    /// Per core frame, per dynamic object: mean-square signal power from the CAF essence.
    powers: Vec<Vec<f32>>,
}

fn load_ctx(core: &Path, atmos: &Path) -> Result<AtmosCtx> {
    let manifest = damf::Manifest::load(atmos)?;
    let dir = atmos.parent().unwrap_or_else(|| Path::new("."));
    let pres = manifest.presentations.first().context("manifest has no presentations")?;

    // MVP: the dynamic-object-only OAMD program supports only an LFE bed.
    let mut bed_channels = 0usize;
    let mut has_lfe = false;
    for b in &pres.bed_instances {
        for c in &b.channels {
            bed_channels += 1;
            if c.channel.eq_ignore_ascii_case("LFE") {
                has_lfe = true;
            }
        }
    }
    anyhow::ensure!(
        bed_channels == 1 && has_lfe,
        "MVP supports an LFE-only bed; found {bed_channels} bed channels (has_lfe={has_lfe})"
    );

    // Per-object position keyframes from the diff-encoded metadata events.
    let meta = damf::Metadata::load(&dir.join(&pres.metadata))?;
    let mut cur: BTreeMap<u32, [f32; 3]> = BTreeMap::new();
    let mut keys: BTreeMap<u32, Vec<(u64, [f32; 3])>> = BTreeMap::new();
    for e in &meta.events {
        let Some(id) = e.id else { continue };
        let sp = e.sample_pos.unwrap_or(0);
        let st = cur.entry(id).or_insert([0.0; 3]);
        if let Some(p) = &e.pos {
            if p.len() == 3 {
                *st = [p[0] as f32, p[1] as f32, p[2] as f32];
            }
        }
        keys.entry(id).or_default().push((sp, *st));
    }
    let timelines: Vec<PosTimeline> = pres
        .objects
        .iter()
        .map(|o| PosTimeline { keys: keys.get(&o.id).cloned().unwrap_or_default() })
        .collect();
    let nobj = timelines.len();

    let data = std::fs::read(core).with_context(|| format!("reading {}", core.display()))?;
    let frames = eac3::parse_frames(&data);
    anyhow::ensure!(!frames.is_empty(), "no E-AC-3 syncframes in {}", core.display());
    let mut start_samples = Vec::with_capacity(frames.len());
    let mut acc = 0u64;
    for f in &frames {
        start_samples.push(acc);
        if f.strmtyp == 0 {
            acc += f.samples() as u64;
        }
    }

    // Per-frame per-object mean-square power from the CAF essence (object i = channel 1+i,
    // after the single LFE bed channel), binned to the core's frame grid for the JOC analysis.
    let caf_path = dir.join(&pres.audio);
    let caf = damf::read_caf_info(&caf_path)?;
    anyhow::ensure!(caf.bits_per_channel == 24 && !caf.is_float(), "expected 24-bit int CAF essence");
    let channels = caf.channels as usize;
    anyhow::ensure!(channels >= 1 + nobj, "CAF has {channels} channels, need {}", 1 + nobj);
    let mut powers = vec![vec![0f32; nobj]; frames.len()];
    {
        use std::io::{Read, Seek, SeekFrom};
        let mut rdr = std::io::BufReader::new(std::fs::File::open(&caf_path)?);
        rdr.seek(SeekFrom::Start(caf.data_offset))?;
        let block = 8192usize;
        let mut buf = vec![0u8; block * channels * 3];
        let mut t: u64 = 0;
        let mut fi = 0usize;
        let mut remaining = caf.frames();
        'outer: while remaining > 0 {
            let this = remaining.min(block as u64) as usize;
            rdr.read_exact(&mut buf[..this * channels * 3])
                .with_context(|| format!("reading essence at sample {t}"))?;
            for s in 0..this {
                while fi + 1 < frames.len() && t >= start_samples[fi + 1] {
                    fi += 1;
                }
                if t >= start_samples[fi] + frames[fi].samples() as u64 {
                    break 'outer; // essence runs past the last core frame
                }
                let base = s * channels * 3;
                for o in 0..nobj {
                    let off = base + (1 + o) * 3;
                    let v = render::s24be(&buf[off..off + 3]);
                    powers[fi][o] += v * v;
                }
                t += 1;
            }
            remaining -= this as u64;
        }
        for (i, f) in frames.iter().enumerate() {
            let n = f.samples() as f32;
            for v in powers[i].iter_mut() {
                *v /= n;
            }
        }
    }

    Ok(AtmosCtx { timelines, nobj, data, frames, start_samples, powers })
}

/// Per-object 5.1-mains downmix gains at the frame midpoint — the same VBAP pan the `downmix`
/// renderer applied, so the JOC analysis models the core it will reconstruct from.
fn frame_downmix_gains(ctx: &AtmosCtx, i: usize) -> Vec<[f32; 5]> {
    let f = &ctx.frames[i];
    let t = ctx.start_samples[i] + f.samples() as u64 / 2;
    ctx.timelines
        .iter()
        .map(|tl| {
            let p = tl.at(t);
            render::pan_vbap(p[0] as f64, p[1] as f64)
        })
        .collect()
}

/// Sample the per-frame object positions (ramp target = frame end) for frame `i`.
fn frame_objects(ctx: &AtmosCtx, i: usize) -> Vec<emdf::ObjectPos> {
    let f = &ctx.frames[i];
    let t = ctx.start_samples[i] + f.samples() as u64;
    ctx.timelines
        .iter()
        .map(|tl| {
            let p = tl.at(t);
            emdf::ObjectPos { x: p[0], y: p[1], z: p[2] }
        })
        .collect()
}

fn atmos_inject(core: &Path, atmos: &Path, out: &Path) -> Result<()> {
    let ctx = load_ctx(core, atmos)?;
    let object_count = ctx.nobj + 1; // + LFE bed
    let complexity = object_count.min(16) as u8;
    // addbsi §8.3: reserved(7)=0 · flag_ec3_extension_type_a(1)=1 · complexity_index_type_a(8).
    let addbsi = [0x01u8, complexity];

    let mut out_buf = Vec::with_capacity(ctx.data.len() * 11 / 10);
    let mut emdf_bytes = 0usize;
    let mut skipfield_frames = 0usize;
    for (i, f) in ctx.frames.iter().enumerate() {
        let frame = &ctx.data[f.offset..f.offset + f.size];
        let objs = frame_objects(&ctx, i);
        let oamd = emdf::encode_oamd(&objs, true);
        let dmx = frame_downmix_gains(&ctx, i);
        let mtx = joc::analyze_frame(&dmx, &ctx.powers[i]);
        let jocp = joc::encode_joc(&mtx, (i % 1024) as u16);
        // Real Dolby DD+ JOC streams carry two small constant companion payloads after JOC
        // (id 2 = [04 94 80], id 1 = [02 00]); a decoder may require these to treat the stream as
        // object audio. Match the working EAC3-JOC ATMOS file's order: OAMD(11), JOC(14), 2, 1.
        let payload = emdf::wrap_emdf(&[
            (emdf::PAYLOAD_OAMD, oamd),
            (emdf::PAYLOAD_JOC, jocp),
            (2, vec![0x04, 0x94, 0x80]),
            (1, vec![0x02, 0x00]),
        ]);
        emdf_bytes += payload.len();
        // Prefer real Dolby-style carriage: EMDF in an audio-block skip field (mid-frame). Fall
        // back to tail auxdata if a frame's audio blocks are outside the walker's supported subset.
        let injected = match eac3::inject_frame_skipfield(frame, f, &addbsi, &payload, 2) {
            Some(out) => {
                skipfield_frames += 1;
                out
            }
            None => eac3::inject_frame_full(frame, f, &addbsi, &payload, payload.len() * 8),
        };
        out_buf.extend_from_slice(&injected);
    }
    std::fs::write(out, &out_buf).with_context(|| format!("writing {}", out.display()))?;

    // Self-verify: read the addbsi flag, OAMD and JOC back from every output frame. The EMDF is
    // bit-scanned anywhere in the frame (skip-field carriage is not byte-aligned).
    let outframes = eac3::parse_frames(&out_buf);
    let (mut flag_ok, mut oamd_ok, mut joc_ok, checked) = (0usize, 0usize, 0usize, outframes.len());
    for f in &outframes {
        let frame = &out_buf[f.offset..f.offset + f.size];
        if let Some((flag, cx)) = eac3::read_addbsi(frame, f) {
            if flag && cx as usize == object_count {
                flag_ok += 1;
            }
        }
        if let Some((_, aligned)) = joc::find_emdf_anywhere(frame) {
            if let Some(d) = emdf::decode_emdf_oamd(&aligned) {
                if d.object_count == object_count {
                    oamd_ok += 1;
                }
            }
            if let Some(payloads) = emdf::extract_emdf_payloads(&aligned) {
                if let Some((_, jp)) = payloads.iter().find(|(id, _)| *id == emdf::PAYLOAD_JOC) {
                    if let Ok(jf) = joc::decode_joc(jp) {
                        if jf.object_count == ctx.nobj {
                            joc_ok += 1;
                        }
                    }
                }
            }
        }
    }

    let grew = out_buf.len() as i64 - ctx.data.len() as i64;
    println!(
        "Atmos inject: addbsi flag (complexity={complexity}) + OAMD ({} dyn + 1 LFE) + JOC ({} obj, avg {} B EMDF/frame) → {} frames",
        ctx.nobj, ctx.nobj, emdf_bytes / ctx.frames.len(), outframes.len()
    );
    println!(
        "  carriage: skip-field {}/{} frames (rest fell back to auxdata)",
        skipfield_frames, ctx.frames.len()
    );
    println!(
        "  size {} -> {} B (+{} B, +{:.1}%)",
        ctx.data.len(),
        out_buf.len(),
        grew,
        grew as f64 / ctx.data.len() as f64 * 100.0
    );
    println!(
        "  addbsi flag OK on {flag_ok}/{checked} frames; OAMD round-trip on {oamd_ok}/{checked}; JOC round-trip on {joc_ok}/{checked}"
    );
    println!("  wrote {}", out.display());
    println!("  validate core: ffmpeg -i {} -f null -", out.display());
    Ok(())
}

/// Definitive core test: splice MY per-frame EMDF into a REAL Dolby core's skip field in place,
/// preserving its coupled audio. Patches addbsi complexity to 14 (my OAMD object_count) so it stays
/// consistent with my metadata. If this plays Atmos on HW, the ffmpeg core was the wall.
fn coregraft(realcore: &Path, myatmos: &Path, out: &Path, complexity: Option<u8>) -> Result<()> {
    let real = std::fs::read(realcore).with_context(|| format!("reading {}", realcore.display()))?;
    let my = std::fs::read(myatmos).with_context(|| format!("reading {}", myatmos.display()))?;
    let rframes = eac3::parse_frames(&real);
    let mframes = eac3::parse_frames(&my);
    anyhow::ensure!(!rframes.is_empty() && !mframes.is_empty(), "empty input");

    let emdf_container = |buf: &[u8]| -> Option<usize> {
        if buf.len() < 4 {
            return None;
        }
        let body = ((buf[2] as usize) << 8) | buf[3] as usize;
        Some(4 + body)
    };

    let mut out_buf = Vec::with_capacity(real.len());
    let (mut spliced, mut copied) = (0usize, 0usize);
    for (i, rf) in rframes.iter().enumerate() {
        let rframe = &real[rf.offset..rf.offset + rf.size];
        let done = (|| {
            let mf = mframes.get(i)?;
            let mframe = &my[mf.offset..mf.offset + mf.size];
            let (emdf_bit, rbuf) = joc::find_emdf_anywhere(rframe)?;
            let native_len = emdf_container(&rbuf)?;
            // Validate the native match is a real EMDF (has JOC).
            if !emdf::extract_emdf_payloads(&rbuf[..native_len.min(rbuf.len())])
                .map(|ps| ps.iter().any(|(id, _)| *id == emdf::PAYLOAD_JOC))
                .unwrap_or(false)
            {
                return None;
            }
            let (_, mbuf) = joc::find_emdf_anywhere(mframe)?;
            let my_len = emdf_container(&mbuf)?;
            if my_len > mbuf.len() || my_len > native_len {
                return None;
            }
            eac3::splice_emdf_into_core(rframe, rf, emdf_bit, native_len, &mbuf[..my_len], complexity)
        })();
        match done {
            Some(o) => {
                spliced += 1;
                out_buf.extend_from_slice(&o);
            }
            None => {
                copied += 1;
                out_buf.extend_from_slice(rframe);
            }
        }
    }
    std::fs::write(out, &out_buf).with_context(|| format!("writing {}", out.display()))?;
    println!("coregraft: {spliced} spliced, {copied} copied-verbatim → {}", out.display());
    Ok(())
}

/// Bisection: graft a reference DD+ Atmos stream's exact frame-0 EMDF container + addbsi into every
/// frame of my core. If the result plays Atmos on HW but my real output doesn't, the bug is in my
/// metadata encoder; if it still fails, the bug is in my core/carriage/addbsi.
fn graft(core: &Path, reference: &Path, out: &Path) -> Result<()> {
    let core_data = std::fs::read(core).with_context(|| format!("reading {}", core.display()))?;
    let ref_data = std::fs::read(reference).with_context(|| format!("reading {}", reference.display()))?;
    let core_frames = eac3::parse_frames(&core_data);
    let ref_frames = eac3::parse_frames(&ref_data);
    anyhow::ensure!(!ref_frames.is_empty(), "no frames in reference");
    anyhow::ensure!(!core_frames.is_empty(), "no frames in core");

    // Pre-extract every reference frame's full EMDF container + raw addbsi, so we can cycle them
    // per-frame (keeps JOC `seq` incrementing naturally instead of repeating one frame).
    let mut ref_emdf: Vec<Vec<u8>> = Vec::new();
    let mut ref_addbsi: Vec<Vec<u8>> = Vec::new();
    for rf in &ref_frames {
        let rframe = &ref_data[rf.offset..rf.offset + rf.size];
        let Some((_, aligned)) = joc::find_emdf_anywhere(rframe) else { continue };
        let body_len = ((aligned[2] as usize) << 8) | aligned[3] as usize;
        // Guard against false 0x5838 matches: real EMDF fits one 9-bit skip field and must decode
        // to a JOC (id 14) payload.
        if 4 + body_len > aligned.len() || 4 + body_len > 480 {
            continue;
        }
        let emdf = aligned[..4 + body_len].to_vec();
        let has_joc = emdf::extract_emdf_payloads(&emdf)
            .map(|ps| ps.iter().any(|(id, _)| *id == emdf::PAYLOAD_JOC))
            .unwrap_or(false);
        if !has_joc {
            continue;
        }
        let Some((_, addbsi)) = eac3::read_addbsi_raw(rframe, rf) else { continue };
        ref_emdf.push(emdf);
        ref_addbsi.push(addbsi);
    }
    anyhow::ensure!(!ref_emdf.is_empty(), "no usable EMDF frames in reference");
    println!(
        "reference: {} EMDF frames, addbsi [{}], EMDF size {}..{} B",
        ref_emdf.len(),
        ref_addbsi[0].iter().map(|x| format!("{x:02x}")).collect::<Vec<_>>().join(" "),
        ref_emdf.iter().map(|e| e.len()).min().unwrap(),
        ref_emdf.iter().map(|e| e.len()).max().unwrap(),
    );

    let mut out_buf = Vec::with_capacity(core_data.len() * 12 / 10);
    let mut ok = 0usize;
    for (i, f) in core_frames.iter().enumerate() {
        let frame = &core_data[f.offset..f.offset + f.size];
        let j = i % ref_emdf.len();
        let (emdf, addbsi) = (&ref_emdf[j], &ref_addbsi[j]);
        match eac3::inject_frame_skipfield(frame, f, addbsi, emdf, 2) {
            Some(o) => {
                ok += 1;
                out_buf.extend_from_slice(&o);
            }
            None => out_buf.extend_from_slice(&eac3::inject_frame_full(frame, f, addbsi, emdf, emdf.len() * 8)),
        }
    }
    std::fs::write(out, &out_buf).with_context(|| format!("writing {}", out.display()))?;
    println!("graft: {}/{} frames via skip-field → {}", ok, core_frames.len(), out.display());
    Ok(())
}

/// One object's position keyframes (absolute sample → DAMF [x,y,z]).
struct PosTimeline {
    keys: Vec<(u64, [f32; 3])>,
}

impl PosTimeline {
    /// Linearly-interpolated position at absolute sample `t`.
    fn at(&self, t: u64) -> [f32; 3] {
        if self.keys.is_empty() {
            return [0.0; 3];
        }
        let idx = self.keys.partition_point(|&(s, _)| s <= t);
        if idx == 0 {
            return self.keys[0].1;
        }
        let (s0, p0) = self.keys[idx - 1];
        if idx < self.keys.len() {
            let (s1, p1) = self.keys[idx];
            if s1 > s0 {
                let f = (t - s0) as f32 / (s1 - s0) as f32;
                return [
                    p0[0] + (p1[0] - p0[0]) * f,
                    p0[1] + (p1[1] - p0[1]) * f,
                    p0[2] + (p1[2] - p0[2]) * f,
                ];
            }
        }
        p0
    }
}

fn oamd_inject(core: &Path, atmos: &Path, out: &Path) -> Result<()> {
    let manifest = damf::Manifest::load(atmos)?;
    let dir = atmos.parent().unwrap_or_else(|| Path::new("."));
    let pres = manifest.presentations.first().context("manifest has no presentations")?;

    // MVP: the dynamic-object-only OAMD program supports only an LFE bed.
    let mut bed_channels = 0usize;
    let mut has_lfe = false;
    for b in &pres.bed_instances {
        for c in &b.channels {
            bed_channels += 1;
            if c.channel.eq_ignore_ascii_case("LFE") {
                has_lfe = true;
            }
        }
    }
    anyhow::ensure!(
        bed_channels == 1 && has_lfe,
        "MVP supports an LFE-only bed; found {bed_channels} bed channels (has_lfe={has_lfe})"
    );

    // Per-object position keyframes from the diff-encoded metadata events.
    let meta = damf::Metadata::load(&dir.join(&pres.metadata))?;
    let mut cur: BTreeMap<u32, [f32; 3]> = BTreeMap::new();
    let mut keys: BTreeMap<u32, Vec<(u64, [f32; 3])>> = BTreeMap::new();
    for e in &meta.events {
        let Some(id) = e.id else { continue };
        let sp = e.sample_pos.unwrap_or(0);
        let st = cur.entry(id).or_insert([0.0; 3]);
        if let Some(p) = &e.pos {
            if p.len() == 3 {
                *st = [p[0] as f32, p[1] as f32, p[2] as f32];
            }
        }
        keys.entry(id).or_default().push((sp, *st));
    }
    let timelines: Vec<PosTimeline> = pres
        .objects
        .iter()
        .map(|o| PosTimeline { keys: keys.get(&o.id).cloned().unwrap_or_default() })
        .collect();
    let nobj = timelines.len();

    // Load the core stream and map each frame to its start sample (advance on independent frames).
    let data = std::fs::read(core).with_context(|| format!("reading {}", core.display()))?;
    let frames = eac3::parse_frames(&data);
    anyhow::ensure!(!frames.is_empty(), "no E-AC-3 syncframes in {}", core.display());
    let mut start_samples = Vec::with_capacity(frames.len());
    let mut acc = 0u64;
    for f in &frames {
        start_samples.push(acc);
        if f.strmtyp == 0 {
            acc += f.samples() as u64;
        }
    }

    // Inject a per-frame OAMD payload. Sample positions at frame end (ramp target).
    let injected = eac3::inject_stream(&data, |f, i| {
        let t = start_samples[i] + f.samples() as u64;
        let objs: Vec<emdf::ObjectPos> = timelines
            .iter()
            .map(|tl| {
                let p = tl.at(t);
                emdf::ObjectPos { x: p[0], y: p[1], z: p[2] }
            })
            .collect();
        let payload = emdf::encode_frame_emdf(&objs, true);
        let bits = payload.len() * 8;
        Some((payload, bits))
    });
    std::fs::write(out, &injected).with_context(|| format!("writing {}", out.display()))?;

    // Self-verify: round-trip the OAMD back out of every output frame.
    let outframes = eac3::parse_frames(&injected);
    let (mut ok, mut checked) = (0usize, 0usize);
    for f in &outframes {
        let frame = &injected[f.offset..f.offset + f.size];
        if let Some((aux, _)) = eac3::read_aux(frame) {
            if let Some(d) = emdf::decode_emdf_oamd(&aux) {
                checked += 1;
                if d.object_count == nobj + 1 && d.bed_count == 1 {
                    ok += 1;
                }
            }
        }
    }

    let grew = injected.len() as i64 - data.len() as i64;
    println!(
        "OAMD inject: {nobj} dynamic objects + 1 LFE bed → {} frames",
        outframes.len()
    );
    println!(
        "  size {} -> {} B (+{} B, +{:.1}%)",
        data.len(),
        injected.len(),
        grew,
        grew as f64 / data.len() as f64 * 100.0
    );
    println!("  OAMD round-trip OK on {ok}/{checked} frames (object_count={}, bed=1)", nobj + 1);
    println!("  wrote {}", out.display());
    println!("  validate core: ffmpeg -i {} -f null -", out.display());
    Ok(())
}

fn eac3inject(input: &Path, out: &Path, marker: &str) -> Result<()> {
    let payload = hex_bytes(marker).context("parsing --marker as hex")?;
    anyhow::ensure!(!payload.is_empty(), "marker payload is empty");
    let bits = payload.len() * 8;

    let data = std::fs::read(input).with_context(|| format!("reading {}", input.display()))?;
    let before = eac3::parse_frames(&data);
    anyhow::ensure!(!before.is_empty(), "no E-AC-3 syncframes in {}", input.display());

    // Inject the marker into every frame.
    let injected = eac3::inject_stream(&data, |_f, _i| Some((payload.clone(), bits)));
    std::fs::write(out, &injected).with_context(|| format!("writing {}", out.display()))?;

    // Self-verify: re-parse the output, confirm frame count is preserved and the
    // marker round-trips out of the aux field of every frame.
    let after = eac3::parse_frames(&injected);
    anyhow::ensure!(
        after.len() == before.len(),
        "frame count changed after injection: {} -> {}",
        before.len(),
        after.len()
    );
    let mut ok = 0usize;
    for f in &after {
        let frame = &injected[f.offset..f.offset + f.size];
        match eac3::read_aux(frame) {
            Some((got, got_bits)) if got_bits == bits && got[..payload.len()] == payload[..] => {
                ok += 1
            }
            other => anyhow::bail!("aux round-trip failed at frame off {}: {:?}", f.offset, other),
        }
    }

    let grew = injected.len() as i64 - data.len() as i64;
    println!("injected marker {} ({} B) into {} frames", marker, payload.len(), after.len());
    println!(
        "  size {} -> {} B (+{} B, +{:.1}%)",
        data.len(),
        injected.len(),
        grew,
        grew as f64 / data.len() as f64 * 100.0
    );
    println!("  aux round-trip verified on all {ok} frames");
    println!("  wrote {}", out.display());
    println!("  validate decode: ffmpeg -i {} -f null -", out.display());
    Ok(())
}

/// Parse a hex string (e.g. "DEADBEEF" or "de ad be ef") into bytes.
fn hex_bytes(s: &str) -> Result<Vec<u8>> {
    let clean: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    anyhow::ensure!(clean.len() % 2 == 0, "hex string must have an even number of digits");
    (0..clean.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&clean[i..i + 2], 16).context("invalid hex digit"))
        .collect()
}

fn eac3probe(input: &Path) -> Result<()> {
    let data = std::fs::read(input).with_context(|| format!("reading {}", input.display()))?;
    let frames = eac3::parse_frames(&data);
    anyhow::ensure!(!frames.is_empty(), "no E-AC-3 syncframes found in {}", input.display());

    let mut by_sub: BTreeMap<(u8, u8), usize> = BTreeMap::new();
    for f in &frames {
        *by_sub.entry((f.strmtyp, f.substreamid)).or_default() += 1;
    }
    let f0 = &frames[0];
    let total_bytes: usize = frames.iter().map(|f| f.size).sum();
    let independent = frames.iter().filter(|f| f.strmtyp == 0).count();
    let duration = independent as f64 * f0.samples() as f64 / f0.sample_rate.max(1) as f64;

    println!("E-AC-3 stream : {}", input.display());
    println!("  syncframes  : {} ({} bytes)", frames.len(), total_bytes);
    for ((st, sid), n) in &by_sub {
        let label = match st {
            0 => "independent",
            1 => "dependent",
            2 => "AC-3-conv",
            _ => "reserved",
        };
        println!("  substream {sid} ({label}): {n} frames");
    }
    let codec = if f0.bsid == 16 {
        "E-AC-3"
    } else if f0.bsid <= 8 {
        "AC-3"
    } else {
        "?"
    };
    println!(
        "  first frame : bsid {} ({}), acmod {} ({} ch{}), {} blocks, {} Hz, {} B",
        f0.bsid,
        codec,
        f0.acmod,
        f0.full_channels(),
        if f0.lfe { "+LFE" } else { "" },
        f0.blocks,
        f0.sample_rate,
        f0.size,
    );
    println!(
        "  duration    : {duration:.2}s ({independent} independent frames × {} samples)",
        f0.samples()
    );

    // Atmos signalling analysis: addbsi flag + EMDF payloads (OAMD=11, JOC=14).
    println!("\nAtmos signalling (first 3 frames):");
    for f in frames.iter().take(3) {
        let frame = &data[f.offset..f.offset + f.size];
        let addbsi = eac3::read_addbsi(frame, f);
        let raw = eac3::read_addbsi_raw(frame, f);
        let emdf = joc::find_emdf_anywhere(frame).and_then(|(_, b)| emdf::list_emdf_payloads(&b));
        println!(
            "  off {:>7}: addbsi={:?}  addbsi_raw={}  emdf_payloads={:?}",
            f.offset,
            addbsi,
            raw.as_ref()
                .map(|(n, b)| format!("{n}B [{}]", b.iter().map(|x| format!("{x:02x}")).collect::<Vec<_>>().join(" ")))
                .unwrap_or_else(|| "none".into()),
            emdf,
        );
        // Dump raw bytes of each EMDF payload (to inspect ids 1/2 companion payloads).
        if let Some((_, b)) = joc::find_emdf_anywhere(frame) {
            if let Some(ps) = emdf::extract_emdf_payloads(&b) {
                for (id, bytes) in &ps {
                    println!(
                        "        payload id={id} [{}]",
                        bytes.iter().map(|x| format!("{x:02x}")).collect::<Vec<_>>().join(" ")
                    );
                }
            }
        }
    }
    let (mut n_flag, mut n_aux, mut n_oamd, mut n_joc) = (0usize, 0, 0, 0);
    for f in &frames {
        let frame = &data[f.offset..f.offset + f.size];
        if let Some((true, _)) = eac3::read_addbsi(frame, f) {
            n_flag += 1;
        }
        if let Some((b, _)) = eac3::read_aux(frame) {
            n_aux += 1;
            if let Some((_, ps)) = emdf::list_emdf_payloads(&b) {
                if ps.iter().any(|(id, _)| *id == 11) {
                    n_oamd += 1;
                }
                if ps.iter().any(|(id, _)| *id == 14) {
                    n_joc += 1;
                }
            }
        }
    }
    println!(
        "  totals over {} frames: addbsi-flag={n_flag}  aux-present={n_aux}  OAMD={n_oamd}  JOC={n_joc}",
        frames.len()
    );
    Ok(())
}

fn inspect(atmos: &Path) -> Result<()> {
    let manifest = damf::Manifest::load(atmos)?;
    let dir = atmos.parent().unwrap_or_else(|| Path::new("."));
    let pres = manifest
        .presentations
        .first()
        .context("manifest has no presentations")?;

    println!("DAMF version : {}", manifest.version);

    let bed: Vec<_> = pres
        .bed_instances
        .iter()
        .flat_map(|b| b.channels.iter())
        .collect();
    println!("Bed channels : {}", bed.len());
    for c in &bed {
        println!("  - {:<5} (ID {})", c.channel, c.id);
    }
    let obj_ids: Vec<u32> = pres.objects.iter().map(|o| o.id).collect();
    println!("Objects      : {}  IDs {:?}", obj_ids.len(), obj_ids);

    let total = bed.len() + pres.objects.len();
    println!(
        "Elements     : {}  -> JOC clustering {} ({} objects, max 16)",
        total,
        if pres.objects.len() <= 16 {
            "NOT needed"
        } else {
            "REQUIRED"
        },
        pres.objects.len()
    );

    // Audio essence.
    let audio_path = dir.join(&pres.audio);
    let caf = damf::read_caf_info(&audio_path)?;
    let fmt = std::str::from_utf8(&caf.format_id).unwrap_or("?");
    println!("\nAudio essence: {}", audio_path.display());
    println!(
        "  {} ch, {}{}-bit {}, {} Hz, {} frames ({:.2}s), fmt '{}' flags {:#x}",
        caf.channels,
        if caf.is_float() { "float " } else { "" },
        caf.bits_per_channel,
        if caf.is_big_endian() { "BE" } else { "LE" },
        caf.sample_rate,
        caf.frames(),
        caf.frames() as f64 / caf.sample_rate,
        fmt,
        caf.format_flags,
    );
    if caf.channels as usize == total {
        println!("  ✓ channel count matches element count (bed first, then objects)");
    } else {
        println!(
            "  ⚠ channel count {} != element count {} — verify channel mapping",
            caf.channels, total
        );
    }

    // Metadata timeline.
    let meta_path = dir.join(&pres.metadata);
    let meta = damf::Metadata::load(&meta_path)?;
    let mut events_per_id: BTreeMap<u32, usize> = BTreeMap::new();
    let mut max_pos = 0u64;
    for e in &meta.events {
        if let Some(id) = e.id {
            *events_per_id.entry(id).or_default() += 1;
        }
        if let Some(p) = e.sample_pos {
            max_pos = max_pos.max(p);
        }
    }
    println!("\nMetadata     : {}", meta_path.display());
    println!(
        "  {} events over {} elements; timeline 0..{} samples ({:.2}s)",
        meta.events.len(),
        events_per_id.len(),
        max_pos,
        max_pos as f64 / caf.sample_rate,
    );
    let movers: Vec<String> = events_per_id
        .iter()
        .filter(|&(_, &n)| n > 1)
        .map(|(id, n)| format!("ID{id}×{n}"))
        .collect();
    if movers.is_empty() {
        println!("  all elements static in this segment");
    } else {
        println!("  dynamic elements: {}", movers.join(", "));
    }

    Ok(())
}
