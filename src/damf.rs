//! Reader for the Dolby Atmos Master File set (DAMF) produced by `truehdd decode`:
//! a `.atmos` YAML manifest, a `.atmos.metadata` YAML event list, and a `.atmos.audio`
//! CAF essence (bed channels + object channels). We only model the subset we need to
//! encode E-AC-3 JOC; unknown YAML fields are ignored.

use anyhow::{Context, Result, ensure};
use serde::Deserialize;
use std::fs::File;
use std::io::{BufReader, Read, Seek, SeekFrom};
use std::path::Path;

/// Top-level `.atmos` manifest.
#[derive(Debug, Deserialize)]
pub struct Manifest {
    pub version: String,
    pub presentations: Vec<Presentation>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Presentation {
    /// Filename of the `.atmos.metadata` companion (relative to the manifest dir).
    pub metadata: String,
    /// Filename of the `.atmos.audio` companion (relative to the manifest dir).
    pub audio: String,
    #[serde(default)]
    pub bed_instances: Vec<BedInstance>,
    #[serde(default)]
    pub objects: Vec<ObjectRef>,
}

#[derive(Debug, Deserialize)]
pub struct BedInstance {
    #[serde(default)]
    pub channels: Vec<ChannelRef>,
}

#[derive(Debug, Deserialize)]
pub struct ChannelRef {
    pub channel: String,
    #[serde(rename = "ID")]
    pub id: u32,
}

#[derive(Debug, Deserialize)]
pub struct ObjectRef {
    #[serde(rename = "ID")]
    pub id: u32,
}

impl Manifest {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("reading manifest {}", path.display()))?;
        serde_yaml_ng::from_str(&text)
            .with_context(|| format!("parsing manifest {}", path.display()))
    }
}

/// `.atmos.metadata`: sample rate + a flat, diff-encoded list of per-element events.
/// The first events (at `samplePos: 0`) carry full state; later entries carry only the
/// fields that changed for a given element ID.
#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Metadata {
    pub sample_rate: Option<u32>,
    #[serde(default)]
    pub events: Vec<Event>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Event {
    #[serde(rename = "ID")]
    pub id: Option<u32>,
    pub sample_pos: Option<u64>,
    pub active: Option<bool>,
    /// Room-anchored position `[x, y, z]` (x: left -1 .. right +1, y: back -1 .. front +1,
    /// z: floor 0 .. ceiling 1). Absent for bed/LFE elements.
    pub pos: Option<Vec<f64>>,
}

impl Metadata {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("reading metadata {}", path.display()))?;
        serde_yaml_ng::from_str(&text)
            .with_context(|| format!("parsing metadata {}", path.display()))
    }
}

/// Header info for the CAF `.atmos.audio` essence (interleaved PCM, bed channels first
/// then objects, in manifest order).
#[derive(Debug, Clone)]
pub struct CafInfo {
    pub sample_rate: f64,
    pub format_id: [u8; 4],
    pub format_flags: u32,
    pub bytes_per_packet: u32,
    pub frames_per_packet: u32,
    pub channels: u32,
    pub bits_per_channel: u32,
    /// Byte offset of the first audio sample (after the `data` chunk's edit-count field).
    pub data_offset: u64,
    /// Number of audio payload bytes (excludes the 4-byte edit count).
    pub data_bytes: u64,
}

impl CafInfo {
    pub fn frames(&self) -> u64 {
        self.data_bytes / self.bytes_per_packet.max(1) as u64
    }
    /// CAF `lpcm` format flags: bit0 = float, bit1 = little-endian.
    pub fn is_float(&self) -> bool {
        self.format_flags & 0x1 != 0
    }
    pub fn is_big_endian(&self) -> bool {
        self.format_flags & 0x2 == 0
    }
}

fn read_be_u32(r: &mut impl Read) -> Result<u32> {
    let mut b = [0u8; 4];
    r.read_exact(&mut b)?;
    Ok(u32::from_be_bytes(b))
}
fn read_be_i64(r: &mut impl Read) -> Result<i64> {
    let mut b = [0u8; 8];
    r.read_exact(&mut b)?;
    Ok(i64::from_be_bytes(b))
}
fn read_be_f64(r: &mut impl Read) -> Result<f64> {
    let mut b = [0u8; 8];
    r.read_exact(&mut b)?;
    Ok(f64::from_be_bytes(b))
}

/// Parse the CAF file header and `desc`/`data` chunk headers. Does not read samples.
pub fn read_caf_info(path: &Path) -> Result<CafInfo> {
    let mut f =
        BufReader::new(File::open(path).with_context(|| format!("open {}", path.display()))?);

    let mut magic = [0u8; 4];
    f.read_exact(&mut magic)?;
    ensure!(&magic == b"caff", "not a CAF file: {}", path.display());
    f.seek(SeekFrom::Current(4))?; // version (2) + flags (2)

    let mut desc: Option<CafInfo> = None;
    loop {
        let mut ctype = [0u8; 4];
        if f.read_exact(&mut ctype).is_err() {
            break; // clean EOF
        }
        let csize = read_be_i64(&mut f)?;
        match &ctype {
            b"desc" => {
                let sample_rate = read_be_f64(&mut f)?;
                let mut fmt = [0u8; 4];
                f.read_exact(&mut fmt)?;
                let format_flags = read_be_u32(&mut f)?;
                let bytes_per_packet = read_be_u32(&mut f)?;
                let frames_per_packet = read_be_u32(&mut f)?;
                let channels = read_be_u32(&mut f)?;
                let bits_per_channel = read_be_u32(&mut f)?;
                desc = Some(CafInfo {
                    sample_rate,
                    format_id: fmt,
                    format_flags,
                    bytes_per_packet,
                    frames_per_packet,
                    channels,
                    bits_per_channel,
                    data_offset: 0,
                    data_bytes: 0,
                });
                if csize > 32 {
                    f.seek(SeekFrom::Current(csize - 32))?;
                }
            }
            b"data" => {
                let _edit_count = read_be_u32(&mut f)?;
                let pos = f.stream_position()?;
                let data_bytes = if csize < 0 {
                    let end = f.seek(SeekFrom::End(0))?;
                    end - pos
                } else {
                    (csize as u64).saturating_sub(4)
                };
                let info = desc.as_mut().context("CAF 'data' chunk before 'desc'")?;
                info.data_offset = pos;
                info.data_bytes = data_bytes;
                break;
            }
            _ => {
                ensure!(csize >= 0, "negative size for chunk {:?}", ctype);
                f.seek(SeekFrom::Current(csize))?;
            }
        }
    }
    desc.context("CAF missing 'desc' chunk")
}
