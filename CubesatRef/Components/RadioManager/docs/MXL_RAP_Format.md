# MXL Radio Packet Formats

Transcribed from "MXL Tech Memo — MXL Radio Packet Formats" (Revision as on GRIFEX, October 24, 2024).
Authors: James Cutler (jwculter@umich.edu), Collin Hockey (hockeyc@umich.edu), Jin Xin Ng (jxn@umich.edu).
Original packet definitions developed in the Space Systems Development Laboratory (SSDL) at Stanford University.

`RadioManager::TRANSMIT_PACKET_cmdHandler` (`RadioManager.cpp`) implements the RAP header
and wraps its payload as a DAP, per this spec. See the notes at the end of this doc for
where the current implementation deviates or simplifies.

## Communication Format Details

Flight CPUs used by MXL (MSP430, armv5tej) store information in little endian (least
significant byte first) format. Integers are stored and transmitted this way. Signed
integers are stored in 2's complement.

## Radio Application Packet (RAP)

Radio packets are transmitted over wireless links. Radio packets contain Telemetry,
Command, and Data Packets in their data section.

| Field | Size (bytes) | Description |
|---|---|---|
| Sync Characters | 2 | Hex: `AB CD`. Decimal: 171, 205. |
| Primary ID (Satellite/Org) | 2 | Address space for 65536 satellites/organizations. |
| Secondary ID (Application ID) | 2 | Secondary sorting when multiple applications share a channel. **Big endian** (the one field that deviates from the general little-endian rule). |
| Flags | 1 | Packet characterization — see below. |
| Length | 2 | Total bytes in the entire radio packet, including sync and checksums. |
| Header Checksum | 2 | FLETCHER 16 over Sync + IDs + Flags + Length (sync chars included). |
| Data | ≤238 | Exactly one type of inner packet. |
| Checksum | 2 | FLETCHER 16 over the Data section. |
| HMAC | 4 | First 4 bytes of HMAC-SHA1 over the packet. |

### Flags

Not bit flags — lower 4 bits and upper 4 bits mean different things.

**Lower 4 bits — packet type** (exactly one type of packet per RAP):
| Value | Type |
|---|---|
| `0x0` | TAP — Telemetry Application Packet |
| `0x1` | DAP — Data Application Packet |
| `0x2` | CAP — Command Application Packet |
| `0x3` | Beacon |
| `0x4` | EAP — Event Application Packet |
| `0x5` | ACK |

**Upper 4 bits — action:**
| Value | Action |
|---|---|
| `0x8` | ACK REQ — request the receiver acknowledge this packet |

Example: a DAP that requests acknowledgement → Flags = `0x81`.

## Data Application Packet (DAP)

Used for large blocks of data (payload data, files). NACK-based: request a block,
track received packets, respond with what's missing.

| Field | Size (bytes) | Description |
|---|---|---|
| Primary ID | 1 | ID of the onboard "application"/process this packet is for. |
| Length | 2 | Length of this DAP, including its own header. |
| File Number | 2 | Up to 65,535 files. |
| File Part | 2 | Up to 65,535 parts per file. |
| Total Parts in File | 2 | Total expected parts for this file. |
| Data | ≤229 | Payload bytes. |

## Command Application Packet (CAP)

| Field | Size (bytes) | Description |
|---|---|---|
| Primary ID | 1 | ID of the command receiver ("application" process). |
| Length | 2 | Length of the command packet. |
| Command Reference Number | 2 | Incrementing number, used to reference this command later (e.g. in an ack). |
| Execution Time | 4 | Unix timestamp (UTC) to execute the command. |
| Command Name | 2 | Command to execute. |
| Arguments | ≤225 | Command-specific arguments. |
| Checksum | 2 | FLETCHER 16. |

## Telemetry Application Packet (TAP) — not verified as of 10/24/22

Snapshot of one or many same-type telemetry points sampled at the same time. Good for
beacons; DAP is recommended for bulk/efficient telemetry storage instead.

| Field | Size (bytes) | Description |
|---|---|---|
| Primary ID | 1 | ID of packet, tied to an onboard "application" process. |
| Secondary ID | 1 | Characterizes the packet (e.g. rapid-fire mode or not). |
| TAP Size | 2 | Total bytes in the TAP. |
| Timestamp | 4 | UTC seconds (lasts until 2038). |
| Telemetry Records | <1014 | Format defined per Primary ID. Normal mode: each point listed under the global timestamp. Rapid-fire mode: each point prefaced by a 1-byte sub-second timestamp (units of 1/256s), wrapping detects second boundaries. |
| Checksum | 2 | FLETCHER 16. |

## Event Application Packet (EAP) — not verified as of 10/24/22

Log of unusual spacecraft events/errors. Codes are defined in common headers and mapped
to messages in the mission database.

| Field | Size (bytes) | Description |
|---|---|---|
| EAP Size | 2 | Total bytes in the EAP. |
| Event Records | <1020 | Each record: 4-byte timestamp + 2-byte error code + 2-byte error value = 8 bytes/event. |
| Checksum | 2 | FLETCHER 16. |

## Acknowledgements

| Field | Size (bytes) | Description |
|---|---|---|
| Sender HMAC | 4 | HMAC of the RAP that triggered this ACK. |

## Beacon Information

Beacons are a plain data dump inside a RAP — not wrapped in any inner packet, to save
bytes since beacon packets aren't re-sent if missed. All FM430 data is integer format
(2's complement); no floating point. Protocol specifics live on MIDAS's craft
configuration page (not reproduced here — mission-specific).

## Communication Concept

For a pass (or set of passes), downlink data is scheduled. The satellite sends the
scheduled packets; the ground station tracks what was received vs. missed. On the next
opportunity, the ground station acks the highest-numbered packet received plus any gaps
before it; the satellite resumes with the missed packets first, then continues with new
ones.

---

## Deviations in this implementation

`RadioManager::TRANSMIT_PACKET_cmdHandler` implements the RAP header and wraps its
payload as a DAP. Known simplifications, to revisit before this touches a real link:

- **HMAC is stubbed to 4 zero bytes.** Real HMAC-SHA1 needs a pre-shared key and new
  crypto infrastructure — this repo currently only builds CRC32 (see `lib/fprime/Utils/Hash`;
  an OpenSSL SHA256 binding exists in source but isn't wired into that module's
  `CMakeLists.txt`). No SHA1 implementation exists anywhere in this repo yet.
- **`RAP_PRIMARY_ID`, `RAP_SECONDARY_ID`, `DAP_PRIMARY_ID` are placeholders** (`0x0001`,
  `0x0001`, `0x01`) — real values come from whatever ground-station/mission registry
  assigns satellite and application IDs.
- **No fragmentation.** `TRANSMIT_PACKET` sends one DAP per call with `File Number=0`,
  `File Part=0`, `Total Parts=1` — a single generic buffer, not real multi-part file
  transfer. Data over 229 bytes is rejected (`VALIDATION_ERROR`), not split across
  multiple DAPs.
- **No RX-side parsing.** This only covers the transmit path — there's no RAP decode
  (sync detection, checksum validation) for incoming packets yet.
- **Only DAP is implemented.** TAP/CAP/EAP/Beacon/ACK framing don't exist in
  `RadioManager` — `TRANSMIT_PACKET`'s Flags are hardcoded to `0x1` (DAP, no ack
  requested).
