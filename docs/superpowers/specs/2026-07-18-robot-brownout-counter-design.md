# Robot Brownout Counter Design

Status: Approved by the user on 2026-07-18

## Purpose

Create one standalone Windows PowerShell script that reports how many times a robot browned out in:

- an AdvantageKit/WPILib `.wpilog`;
- one raw CTRE `.hoot`;
- the paired raw Hoot files produced for the roboRIO CAN bus and a CANivore; or
- a directory containing any of those files.

The user must be able to pass raw `.hoot` files directly. They will not be required to open Phoenix Tuner or manually create an intermediate file.

## Relevant Log Semantics

An AdvantageKit log records the roboRIO brownout state directly as the boolean entry `/SystemStats/BrownedOut`. A false-to-true transition is one exact observed roboRIO brownout.

A CTRE Hoot does not contain that AdvantageKit entry. It contains global robot mode/enable signals and per-controller `Fault_BridgeBrownout` signals. CTRE defines the controller fault as the motor bridge being disabled, most likely because supply voltage dropped too low. Therefore, a Hoot-only robot-level result is necessarily an estimate and must never be presented as exact.

The paired E13 samples demonstrate this distinction:

- The AdvantageKit log contains 44 `/SystemStats/BrownedOut` rising edges.
- The converted Hoot data contains 34 bounded, brief `Teleop -> Disabled -> Teleop` interruptions. Controller bridge-brownout faults provide supporting evidence, but their default sampling behavior cannot reconstruct every short roboRIO brownout.

## Goals

- Provide a single production file: `tools/Count-RobotBrownouts.ps1`.
- Run under Windows PowerShell 5.1 without Python, Gradle, or project compilation.
- Parse WPILOG records directly using only .NET classes.
- Accept raw Hoot input and invoke CTRE's official Owlet decoder internally.
- Automatically combine same-session Hoot files from multiple CAN buses without double-counting their duplicated global mode events.
- Preserve and never modify the input logs.
- Tolerate a partial final WPILOG record and report that the tail was truncated.
- Clearly label each result as `exact` or `estimated`.
- Give actionable errors for unsupported logs, failed conversion, or unavailable downloads.

## Non-Goals

- Reverse-engineer or independently implement CTRE's proprietary Hoot format.
- Claim that a Hoot-derived result is an exact roboRIO brownout count.
- Count every controller fault as a separate robot brownout.
- Add robot-runtime code, a GUI, or changes to the Gradle build.
- Modify, rename, or delete any source log.

## Approaches Considered

### 1. Automatic official Owlet decoding (selected)

The PowerShell script accepts `.hoot` directly, locates or downloads CTRE Owlet, converts into a uniquely named temporary WPILOG, analyzes it, and removes the temporary data.

This preserves the requested one-command workflow while relying on CTRE's supported decoder. It also avoids committing a large third-party executable to the robot repository.

### 2. Require Owlet or Tuner to be installed

This avoids an automatic download but makes setup manual and means the script is not self-starting on a new driver-station laptop. An explicit `-OwletPath` override will still support offline installations, but it is not the default.

### 3. Implement a Hoot decoder in PowerShell

This would remove the external executable but would depend on an undocumented, evolving format. It is rejected as brittle and likely to silently misinterpret logs.

## Command-Line Interface

Examples:

```powershell
.\tools\Count-RobotBrownouts.ps1 '.\match.wpilog'

.\tools\Count-RobotBrownouts.ps1 `
  '.\match_rio_2026-06-27_21-02-40.hoot', `
  '.\match_CANIVORE_2026-06-27_21-02-40.hoot'

.\tools\Count-RobotBrownouts.ps1 '.\TRI Logs\Hoot Logs\TXHOU1_E13'
```

Parameters:

- `-Path <string[]>` is mandatory and accepts files or directories. Directory discovery is recursive.
- `-OwletPath <string>` optionally selects an already downloaded Owlet executable.
- `-Detailed` prints event timestamps, truncation/conversion warnings, and controller bridge-fault evidence.

For one logical log, the normal output leads with one line:

```text
Brownouts: 44 (exact)
```

or:

```text
Brownouts: 34 (estimated from Hoot)
```

Multiple discovered logs receive a source heading and one result each. Errors go to the PowerShell error stream. A run exits nonzero if any requested logical log cannot be analyzed.

## Input Discovery and Hoot Grouping

File inputs are analyzed directly. Directory inputs recursively discover `.wpilog` and `.hoot` files.

Raw Hoot filenames ending in the same `YYYY-MM-DD_HH-mm-ss` timestamp and located in the same directory form one logical Hoot session. This matches CTRE's one-file-per-CAN-bus layout. A filename without that suffix forms its own session.

Within a directory:

- Raw `.hoot` files are always used when present; sibling Hoot-converted `.wpilog` files do not replace them.
- A WPILOG containing `/SystemStats/BrownedOut` is independently reported as an exact AdvantageKit result.
- A WPILOG without the exact entry but with CTRE `RobotMode`, `RobotEnable`, or `Fault_BridgeBrownout` entries can be analyzed as already converted Hoot data.
- A converted Hoot WPILOG is suppressed when its corresponding raw Hoot is already part of the same requested directory scan.

## Data Flow

1. Resolve and validate all requested paths.
2. Discover files and create logical WPILOG or Hoot sessions.
3. For raw Hoot sessions, resolve Owlet and convert every bus log into a temporary WPILOG.
4. Parse only the entries needed for brownout analysis.
5. Sort relevant samples by timestamp and original record order because WPILOG does not guarantee timestamp ordering.
6. Apply the exact WPILOG or estimated Hoot event definition.
7. Deduplicate duplicated Hoot global events across paired bus logs.
8. Print results and warnings.
9. Remove temporary conversion files in a `finally` block.

## WPILOG Reader

The script implements the published WPILib Data Log 1.0 format:

- validate the `WPILOG` magic and version;
- read variable-width entry ID, payload length, and timestamp fields;
- decode Start and Finish control records;
- decode only boolean, double, and string values needed by this tool; and
- stop safely when the final record is incomplete.

The parser returns relevant samples plus a `TruncatedTail` flag. An incomplete tail is a warning when usable earlier records exist, not an automatic failure.

## Exact AdvantageKit Count

The exact analyzer recognizes `SystemStats/BrownedOut` with or without a leading slash.

Samples are sorted by timestamp and stable record sequence. Every transition into `true` counts once. If the first observed sample is already `true`, it counts as an event that was active when logging began. Repeated `true` samples do not increment the count.

The result includes:

- `Count`;
- event timestamps;
- `Method = ExactRoboRioBrownedOut`; and
- any truncated-tail warning.

## Estimated Hoot Count

The preferred Hoot signal is the global string `RobotMode`.

One estimated event is a bounded mode sequence where:

1. the mode changes from `Autonomous`, `Teleop`, or `Test` to `Disabled`;
2. it returns to the same enabled mode within 1.0 second; and
3. both boundaries occur inside the analyzed log.

Requiring the same enabled mode excludes a normal autonomous-to-teleop transition. Requiring a return excludes the final match disable.

If `RobotMode` is unavailable, the analyzer applies the same bounded-interval rule to the global boolean `RobotEnable`: `true -> false -> true` within 1.0 second.

Events from paired Hoot files whose start timestamps are within 25 milliseconds are duplicates and merge into one event. `Fault_BridgeBrownout` rising edges are retained as supporting diagnostics but do not independently increase the robot-level count.

If neither global signal exists, the script reports that a robot-level count cannot be estimated. It may list controller bridge faults in detailed diagnostics, but it does not relabel them as robot brownouts.

The result includes:

- `Count`;
- event timestamps;
- `Method = EstimatedHootEnableInterruptions`;
- the global signal used;
- the number and names of controllers with bridge-brownout evidence; and
- conversion or truncation warnings.

## Owlet Resolution and Integrity

Resolution order:

1. the explicit `-OwletPath`;
2. an `owlet` executable on `PATH`;
3. the tool's per-user cache; and
4. an automatic download from CTRE.

The initial implementation pins the current stable 2026 decoder:

```text
Version: 26.3.0
URL: https://redist.ctr-electronics.com/tools/owlet/26.3.0/owlet-26.3.0-windowsx86-64.exe
Published SHA-1: 01df8c1aa20cb0f6af47d550c860474a30c6be36
```

The cache location is `%LOCALAPPDATA%\FRCBrownoutCounter`. The script verifies the published checksum before executing a downloaded or cached binary. A checksum mismatch deletes the invalid cached download and stops with an error.

Owlet is invoked as:

```text
owlet -f wpilog input.hoot output.wpilog
```

If Owlet returns nonzero but produced a valid WPILOG, the script analyzes the usable data and surfaces the conversion warning. This supports CTRE's documented case where shutdown leaves bad data at the end of a Hoot. If there is no valid output, conversion fails.

## Error Handling

The script reports a clear error for:

- a missing input path;
- a path with no supported files;
- an invalid or unsupported WPILOG header;
- a WPILOG containing neither an exact brownout entry nor supported Hoot signals;
- a missing, invalid, or checksum-mismatched Owlet executable;
- a failed Hoot conversion with no usable output; or
- a Hoot conversion without a global mode/enable signal.

When a directory contains multiple logical logs, valid results are still printed, followed by a nonzero exit if one or more other logs failed.

## Testing

Tests are written before production code and use a PowerShell test harness with no Pester dependency.

Synthetic WPILOG fixtures cover:

- two exact false-to-true transitions;
- an initially true exact signal;
- repeated true samples;
- stable timestamp ordering;
- a partial trailing record;
- short Hoot mode interruptions;
- exclusion of a long autonomous-to-teleop disable;
- exclusion of an unbounded final disable;
- the `RobotEnable` fallback;
- paired-bus event deduplication; and
- unsupported-signal errors.

Owlet resolution tests cover explicit paths, cache hits, and checksum rejection without requiring the network.

End-to-end verification uses the provided files:

- `akit_26-06-27_21-02-44_txhou1_e13.wpilog` must report 44 exact events despite its partial final record.
- The two raw E13 `.hoot` files must be passed directly, decoded by Owlet, combined as one session, and report the observed Hoot estimate without reading their sibling pre-converted WPILOGs.

## Source References

- [WPILib Data Log File Format Specification](https://github.com/wpilibsuite/allwpilib/blob/main/wpiutil/doc/datalog.adoc)
- [CTRE Signal Logging and Owlet conversion](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/signal-logging.html)
- [CTRE Hoot extraction behavior](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tools/log-extractor.html)
- [CTRE Talon bridge-brownout signal description](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/hardware/traits/HasTalonSignals.html)
