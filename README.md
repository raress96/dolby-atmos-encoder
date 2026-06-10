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

## Use

```sh
# 1. Decode TrueHD Atmos -> DAMF master (external tool, https://github.com/truehdd/truehdd):
truehdd decode movie.thd --output-path master       # -> master.atmos / .audio / .metadata

# 2. Encode the DAMF metadata onto an E-AC-3 core -> DD+ JOC:
dolby-atmos-encoder atmos core.eac3 master.atmos --out atmos.eac3
```

Subcommands (run `--help` on each):

| Command | Purpose |
|---|---|
| `inspect <atmos>` | Report a DAMF master's bed/object layout and metadata. |
| `downmix <atmos> --out d.wav` | Render objects to a 5.1 bed WAV. |
| `atmos <core> <atmos> --out o.eac3` | Inject OAMD+JOC Atmos metadata into an E-AC-3 core. |
| `oamd <core> <atmos> --out o.eac3` | OAMD only (no JOC). |
| `coregraft <realcore> <myatmos> --out o.eac3` | Splice our metadata onto a real Dolby core (diagnostic). |
| `graft <core> <reference> --out o.eac3` | Splice Dolby's metadata onto our core (diagnostic). |
| `jocprobe` / `eac3probe` / `walkprobe` / `bsidump` | Stream inspectors. |
| `oamddump <hex>` | Verbose field-by-field OAMD decode. |
| `emdfverify <input>` | Check whether our protection CRC matches a stream's stored `emdf_protection`. |

Global: `--emdf-key <hex|@file>` and `--emdf-key-id <0..7>` (or `DOLBY_EMDF_KEY`) — see
[The signing seam](#the-signing-seam---emdf-key).

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

## Licensing & provenance

- Some DSP / codec math is **ported or adapted from
  [VoidXH/Cavern](https://github.com/VoidXH/Cavern)**, which is licensed for **non-commercial use**.
  This project inherits that restriction: **personal / research use only — no commercial use and no
  redistribution** without a clean-room rewrite and a license review. `publish = false` is set to
  prevent accidental release to crates.io. **Review this before making the repository public.**
- "Dolby", "Dolby Atmos", "Dolby Digital Plus", and "TrueHD" are trademarks of Dolby Laboratories.
  This is an independent, unaffiliated interoperability/research project, not a Dolby product.
- Built against the publicly available ETSI specifications **TS 102 366** (AC-3/E-AC-3 + Annex H
  EMDF) and **TS 103 420** (object audio metadata / JOC).
