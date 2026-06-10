# dolby-atmos-encoder

Convert a **Dolby Atmos Master (DAMF)** — as decoded from a Dolby **TrueHD + Atmos** stream by
[`truehdd`](https://github.com/truehdd/truehdd) — into **E-AC-3 (Dolby Digital Plus) with Joint
Object Coding (JOC)**, i.e. "DD+ Atmos". The aim was to let consumer gear that can't bitstream
TrueHD Atmos (e.g. an LG TV → Denon AVR over eARC) render real object-based Atmos with height.

> ## Status: research-complete, hardware-blocked
> Every stage below is **provably correct by every software oracle available** (ffmpeg 7,
> [VoidXH/Cavern](https://github.com/VoidXH/Cavern)): the output is detected as
> `E-AC-3 (Dolby Digital Plus + Dolby Atmos)`, decodes with the right object count and valid 3D
> object positions (including height), and is CRC-clean. **But it does not engage Atmos on
> Dolby-certified hardware** — playback falls back to Dolby Surround.
>
> The cause is **two independent, proprietary walls** (see [Why it can't fully
> work](#why-it-cant-fully-work)). This repository is the honest, documented artifact of that
> investigation, with a clean seam for the one missing cryptographic piece.

This is for **personal / interoperability research use only.** See
[Licensing & provenance](#licensing--provenance).

---

## What it does (pipeline)

A near-complete reimplementation of the relevant parts of a DD+ Atmos encoder, in Rust:

- **DAMF reader** — parses the Dolby Atmos Master (`.atmos` / `.audio` / `.metadata`).
- **5.1 downmix renderer** — VBAP-pans the Atmos objects to a 5.1 bed (L R C LFE Ls Rs).
- **OAMD encoder** (ETSI TS 103 420) — per-frame Object Audio Metadata: object positions (with
  elevation), bed/LFE, program assignment. Round-trips through our decoder and Cavern.
- **JOC encoder** — Joint Object Coding matrices over the 5-channel core × parameter bands, with
  Dolby's quant/Huffman config. Bit-exact round-trip.
- **EMDF container** (ETSI TS 102 366 Annex H) — wraps OAMD (id 11) + JOC (id 14) with the
  `emdf_protection` field, carried in the E-AC-3 audio-block **skip field** exactly where real Dolby
  streams put it (recomputing `frmsiz` + `crc2`).
- **addbsi signaling** — the `flag_ec3_extension_type_a` + `complexity_index` Atmos-detection flag.

The E-AC-3 **core** itself is encoded externally (e.g. by ffmpeg); this tool injects/synthesizes the
Atmos metadata layer. See `Command` variants below.

---

## Build

```sh
cargo build --release
```

(Stand-alone crate — it does **not** depend on the `truehdd` library; it consumes the DAMF files
`truehdd` produces. The only out-of-the-ordinary deps are `hmac`/`sha2`, used by the EMDF signing
seam.)

### Memory use (large files)

The frame pipeline streams: `eac3::transform_frames_io` reads the E-AC-3 core in a few-MiB rolling
buffer, transforms each syncframe, and writes it straight out, so memory stays bounded regardless of
file length (E-AC-3 frames are ≤4 KB, so a 4 MiB chunk batches ~1000 frames per read). `eac3inject`
and `oamd` are fully streamed (input + output); `atmos` streams its output and still loads the core
once for per-frame JOC context. Fully streaming `atmos`'s input and the (large) master audio essence
is possible future work. Note: this is plain synchronous streaming — `async` would add a runtime
without helping a single-file, CPU-bound batch transform.

## Test files / samples

You need a source that actually contains **Dolby TrueHD + Atmos**. Public sample sources:

- **Kodi samples wiki** — <https://kodi.wiki/view/Samples> (see the Dolby TrueHD / Atmos entries).
- **r/hometheater — "surround sound test files in almost every format"** —
  <https://www.reddit.com/r/hometheater/comments/11qqv95/surround_sound_test_files_in_almost_every_format/>
  (links to TrueHD Atmos, DD+ JOC Atmos and more).

A **TrueHD Atmos** clip is the lossless source you decode to a DAMF master; a known-good **DD+ JOC
Atmos** clip is handy as a reference for `jocprobe` / `emdfverify`.

## Toolchain: from a Blu-ray MKV to a testable stream

Tools: [`truehdd`](https://github.com/truehdd/truehdd), `ffmpeg` (≥5), `mkvextract` / `mkvmerge`
(MKVToolNix), this crate, and optionally Cavern (verification, below).

```sh
# 0. Build this tool.
cargo build --release                              # -> target/release/dolby-atmos-encoder

# 1. Extract the TrueHD elementary stream from the MKV.
#    Find the TrueHD track first:  ffprobe movie.mkv
ffmpeg -i movie.mkv -map 0:a:0 -c copy -f truehd movie.thd
#    (or, by track id:  mkvextract tracks movie.mkv 1:movie.thd)

# 2. Decode TrueHD -> Dolby Atmos Master (DAMF).  --presentation 3 selects the Atmos presentation.
truehdd decode --presentation 3 movie.thd --output-path master
#    -> master.atmos  +  master.atmos.audio  +  master.atmos.metadata

# 3. Render objects to a 5.1 bed, then encode an E-AC-3 core with ffmpeg.
#    Pipe raw f32le straight into ffmpeg — works for any length (no WAV size cap):
dolby-atmos-encoder downmix master.atmos --out - \
  | ffmpeg -f f32le -ar 48000 -ac 6 -i - -c:a eac3 -b:a 768k -f eac3 core.eac3
#    (Short clips only — `--out downmix.wav` then `ffmpeg -i downmix.wav ...` — but a 5.1
#     32-bit-float WAV is capped at 4 GB, i.e. ~58 min at 48 kHz, so prefer the pipe above.)

# 4. Inject per-frame Atmos metadata (OAMD + JOC + the addbsi detection flag) into the core.
dolby-atmos-encoder atmos core.eac3 master.atmos --out atmos.eac3
#    (add  --emdf-key <hex>  to sign the EMDF protection field — see "The signing seam")

# 5. Mux into an MKV alongside your video.
mkvmerge -o out.mkv --no-audio movie.mkv atmos.eac3

# 6. Quick check (full verification with Cavern below).
ffmpeg -i atmos.eac3            # expect: Audio: eac3 (... Dolby Atmos ...)
```

> ⚠️ This yields a stream ffmpeg/Cavern accept as Atmos, but it will **not** engage Atmos on
> Dolby-certified hardware — see [Why it can't fully work](#why-it-cant-fully-work).

### Subcommands

Run `--help` on each for details.

| Command | Purpose |
|---|---|
| `inspect <atmos>` | Report a DAMF master's bed/object layout and metadata. |
| `downmix <atmos> --out d.wav` | Render objects to a 5.1 bed WAV (`--out -` streams raw f32le to stdout — no 4 GB cap, pipe into ffmpeg). |
| `atmos <core> <atmos> --out o.eac3` | Inject OAMD+JOC metadata + the addbsi flag into an E-AC-3 core. |
| `oamd <core> <atmos> --out o.eac3` | OAMD only (no JOC). |
| `coregraft <realcore> <myatmos> --out o.eac3` | Splice our metadata onto a real Dolby core (diagnostic). |
| `graft <core> <reference> --out o.eac3` | Splice Dolby's metadata onto our core (diagnostic). |
| `jocprobe` / `eac3probe` / `walkprobe` / `bsidump` | Stream inspectors. |
| `oamddump <hex>` | Verbose field-by-field OAMD decode. |
| `emdfverify <input>` | Check whether our protection CRC matches a stream's stored `emdf_protection`. |

Global: `--emdf-key <hex|@file>` and `--emdf-key-id <0..7>` (or `DOLBY_EMDF_KEY`) — see
[The signing seam](#the-signing-seam---emdf-key).

## Verifying with Cavern

[Cavern](https://github.com/VoidXH/Cavern) is the open-source decoder we validate against: if Cavern
reports objects, the OAMD/JOC metadata is well-formed. A small harness is included in
[`tools/cavernprobe`](tools/cavernprobe):

```sh
# Needs the .NET 8 SDK and a local Cavern checkout (adjust the path in cavernprobe.csproj).
dotnet build tools/cavernprobe -c Release
dotnet tools/cavernprobe/bin/Release/net8.0/cavernprobe.dll atmos.eac3
```

It prints the channel layout, **`HasObjects`**, and the decoder's full metadata (`object_count`,
`joc_num_objects`, object positions). On a stream from this tool:

```
channels    : 6
HasObjects  : True
[JOC information]
  joc_num_objects (Number of rendered dynamic objects): ...
...
RESULT: object-based audio detected (Atmos objects present).
```

Exit code `0` = objects present, `2` = channel-based only. (ffmpeg ≥5's `eac3` decoder also reports
`+ Dolby Atmos` as a faster first-pass sanity check.)

## Results / verified on

Run end-to-end on a full feature film — a *Logan* (2017) 2160p UHD Blu-ray remux, TrueHD 7.1 + Atmos
(137 min) — on a single WSL2 workstation:

| Stage | Result |
|---|---|
| `truehdd decode` | 9,891,890 TrueHD frames → 395,675,600 samples (full 137 min), 16 GB DAMF essence |
| `downmix --out -` → ffmpeg | 791 MB E-AC-3 core @ 768 kbps (raw-pipe path; no WAV size cap hit) |
| `atmos` inject | 257,602 E-AC-3 frames; **13 dynamic objects + LFE**; JOC avg 213 B EMDF/frame |
| Carriage | skip-field on **257,602 / 257,602** frames (100%); addbsi detection flag on all |
| Output | 809 MB `atmos.eac3` |
| **Cavern** | `HasObjects=True`, **13 objects**, 1 bed instance, 5-channel JOC downmix |

**Memory is bounded by frame size, not file size.** The streaming frame-I/O path
(`eac3::transform_frames_io`, ~4 MiB rolling buffer) holds a fixed working set regardless of length:
a 1.25 GB synthetic core (487,424 frames) through `eac3inject` peaks at **7.9 MB RSS**. The `atmos`
subcommand is the one exception — it loads the input core once for JOC context, so on the 791 MB
Logan core it peaked at **815 MB RSS** (≈ core size; the *output* is still streamed). Output is
byte-identical before and after the streaming refactor (golden SHA-256 verified).

> These are *decoder-side* results: ffmpeg and Cavern both accept the stream as object Atmos. It
> still will **not** engage Atmos on Dolby-certified hardware — see below for why.

---

## Why it can't fully work

Two diagnostic builds bisect the problem. Both pass ffmpeg 7 and Cavern; both fall back to Dolby
Surround on real hardware:

### Wall 1 — the core encoder
`graft` = **our ffmpeg-encoded core + Dolby's genuine metadata** → Dolby Surround. So the **core
itself** is rejected even with perfect metadata. ffmpeg's E-AC-3 encoder is not Dolby-grade (e.g. it
does no channel coupling); there is no open-source Dolby-conformant E-AC-3 core encoder.

### Wall 2 — EMDF keyed authentication (the decisive one)
`coregraft` = **a real Dolby core + our metadata** → Dolby Surround. The metadata is rejected too,
and we traced it to the EMDF `emdf_protection` field, which is a **keyed authentication code**, not
a computable checksum:

1. **The spec says so.** ETSI TS 102 366 v1.4.1 §H.2.2: `key_id` "identifies the **authentication
   key** used to calculate the value of the `protection_bits_primary` and `protection_bits_secondary`
   fields," and that calculation is "**implementation dependent and is not defined in the present
   document**."
2. **Brute force confirms it.** `emdfverify` dumps each real frame's protection bits; an offline
   sweep of **all 256 CRC-8 polynomials × every init/xorout/reflection over 7 candidate
   byte-regions across 8 real Dolby frames found ZERO matches**, and no standard CRC-32 variant
   matched the 32-bit primary. The null result is the signature of a real secret key.
3. **No open decoder computes it.** `truehdd` marks the field `// TODO: HMAC`; Cavern and ffmpeg
   don't validate it at all. Only Dolby's licensed encoder (holding the key) can produce valid
   protection bits; certified hardware validates them.

**Conclusion:** an open-source encoder that produces *hardware-conformant* DD+ JOC Atmos is not
achievable. The audio coding is solved; the format gates playback behind a proprietary cryptographic
signature by design.

---

## The signing seam (`--emdf-key`)

All of the proprietary cryptography is isolated behind **one trait**, `emdf::EmdfProtector`, with one
real implementation point, `emdf::dolby_keyed_mac`:

- **`PublicCrcProtector`** (default) — public CRC-32 / CRC-8. Well-formed but **unsigned**; this is
  the historical behaviour and what you get with no key.
- **`KeyedProtector`** — selected by `--emdf-key <hex>` (or `DOLBY_EMDF_KEY`). Routes to
  `dolby_keyed_mac`, currently an **HMAC-SHA256 stand-in** that proves the wiring end-to-end.

To produce a signature real hardware accepts you need **both**, and even then a caveat:

1. a valid Dolby authentication key for the correct `key_id`, **and**
2. Dolby's **actual** keyed-MAC construction — exact covered byte-range, MAC algorithm, bit/byte
   ordering, truncation, per-`key_id` handling — implemented in `dolby_keyed_mac`. This is
   undocumented (the spec explicitly declines to define it). **A key alone is not sufficient:** we
   know the container *structure* (32-bit primary + 8-bit secondary + 3-bit `key_id`) but not the
   algorithm or which bytes it covers (our CRCs over the obvious regions do not match Dolby).
3. …and a valid signature still does **not** defeat **Wall 1** for a from-scratch file — only the
   `coregraft` path (our metadata on a *real* Dolby core) could benefit.

> This project does **not** contain, ship, or attempt to recover any Dolby key, and it cannot
> reverse a secure keyed MAC from output samples (that is the security guarantee of a MAC). The seam
> exists for completeness and research. Producing conformant Atmos requires Dolby's licensed encoder.

---

## What actually works on your hardware (no encoder needed)

The real limitation was never the format — it was that some players won't *bitstream* TrueHD Atmos.
Bypass it entirely: feed the **original, untouched** TrueHD Atmos straight into the AVR.

```
PC / media streamer (Kodi, JRiver, Shield, Zidoo…) ──HDMI──> Denon AVR (HDMI in) ──HDMI──> TV (video)
```

The AVR decodes the original TrueHD Atmos losslessly, with full object heights — zero conversion,
zero signature problem. The only requirement is routing audio into the AVR directly rather than
through the TV's eARC return.

---

## Layout

```
src/
  main.rs          CLI + command dispatch (clap)
  damf.rs          DAMF (.atmos/.audio/.metadata) reader
  render.rs        Atmos objects -> 5.1 bed (VBAP)
  emdf.rs          EMDF container, OAMD encode/decode, the EmdfProtector signing seam
  joc.rs           Joint Object Coding analysis + payload writer
  joc_tables.rs    JOC quant / Huffman tables
  eac3.rs          E-AC-3 frame parse, BSI, aux/skip-field injection, crc2
  eac3_audblk.rs   A/52 audio-block bit-walker (locates the skip field)
```

---

## Credits, sources & references

This project stands on open specifications and existing projects:

- **[VoidXH/Cavern](https://github.com/VoidXH/Cavern)** — creator Bence Sgánetz
  (<http://en.sbence.hu>). DSP/codec math for the 5.1 rendering, the OAMD/JOC decode logic, and the
  object gain handling was **ported / adapted from Cavern's C# decoder**. Cavern is released under
  its own **non-commercial, share-alike licence** — see [Licensing](#licensing) below.
- **[truehdd](https://github.com/truehdd/truehdd)** (Apache-2.0) — the Dolby TrueHD decoder that
  produces the DAMF (`.atmos` / `.audio` / `.metadata`) master this tool consumes. This crate began
  life as a `truehdd` workspace member before being extracted into its own repository.
- **ETSI TS 102 366** — Digital Audio Compression (AC-3, Enhanced AC-3), including **Annex H** (the
  EMDF extensible-metadata container and the `emdf_protection` field).
- **ETSI TS 103 420** — backwards-compatible object audio coding (OAMD / Joint Object Coding).
- **ATSC A/52** — the AC-3 audio-block syntax underpinning the skip-field bit-walker.
- **[FFmpeg](https://ffmpeg.org)** — used as an external tool for the E-AC-3 core encode and as a
  reference decoder/validator (invoked as a separate process; not linked).
- **Dolby** specifications and the
  [Dolby Encoding Engine](https://github.com/DolbyLaboratories/dolby-encoding-engine) plugin SDK
  were *examined* for context only. **No Dolby source code or keys are used or included here.**

## Licensing

This project **includes code ported / adapted from
[Cavern](https://github.com/VoidXH/Cavern)**, whose licence is **non-commercial and share-alike**
and states that any project including the code remains bound by its terms. Accordingly the entire
repository is released **under the Cavern licence** (see [`LICENSE`](LICENSE)), *not* MIT:

- ✅ free to use, modify, and redistribute **for free**;
- ⛔ **no selling** any part of the original or a modified version;
- ⛔ no advertisements in the software;
- 🔗 Cavern (<https://github.com/VoidXH/Cavern>, creator <http://en.sbence.hu>) is credited and
  linked as the source — in [Credits](#credits-sources--references) and in `LICENSE`;
- public *use* (e.g. screenings) or commercial use requires the original Cavern creator's permission.

If a permissively-licensed (e.g. MIT) version is ever needed, the Cavern-derived portions must first
be **clean-room rewritten** from the ETSI specs without consulting Cavern's source.

`publish = false` is set in `Cargo.toml` to prevent accidental release to crates.io.

"Dolby", "Dolby Atmos", "Dolby Digital Plus", and "TrueHD" are trademarks of Dolby Laboratories.
This is an independent, unaffiliated interoperability / research project, not a Dolby product.
