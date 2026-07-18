# Robot Brownout Counter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build one standalone Windows PowerShell script that counts exact AdvantageKit/WPILOG roboRIO brownouts and estimates brownouts directly from one or more raw CTRE Hoot files.

**Architecture:** `tools/Count-RobotBrownouts.ps1` contains a small WPILOG reader, exact and Hoot-specific analyzers, input grouping, official Owlet acquisition/conversion, and CLI formatting. A dependency-free PowerShell test runner builds synthetic WPILOG fixtures and tests each boundary before the production behavior is added. Raw Hoots are converted only to uniquely named temporary files; source logs remain untouched.

**Tech Stack:** Windows PowerShell 5.1, .NET `System.IO` binary APIs, CTRE Owlet 26.3.0, WPILib Data Log 1.0.

---

## Scope and File Map

Approved specification:

- `docs/superpowers/specs/2026-07-18-robot-brownout-counter-design.md`

Files created:

- `tools/Count-RobotBrownouts.ps1` — the only production artifact and user entry point.
- `tools/tests/Count-RobotBrownouts.Tests.ps1` — dependency-free tests, fixture generation, and safe temporary-directory cleanup.

No Gradle, Java, vendordep, or robot-runtime source file changes are required.

### Stable internal interfaces

The implementation tasks use these names consistently:

```powershell
Read-WpiLogSignals
Get-ExactBrownoutAnalysis
Get-HootEventsFromSignals
Merge-HootEvents
Get-HootBrownoutAnalysis
Get-InputInventory
Get-FileSha1
Resolve-Owlet
Invoke-OwletConversion
Invoke-RobotBrownoutCounter
Format-BrownoutResult
```

Every parsed sample has this shape:

```powershell
[pscustomobject]@{
    Timestamp = [int64]$Timestamp
    Sequence  = [int64]$Sequence
    Value     = $Value
    Name      = [string]$Name
    Source    = [string]$Source
}
```

Every final result has this shape:

```powershell
[pscustomobject]@{
    Source            = [string]$Source
    Count             = [int]$Count
    Confidence        = 'exact' # or 'estimated'
    Method            = [string]$Method
    Signal            = [string]$Signal
    Events            = [object[]]$Events
    FaultControllers  = [string[]]$FaultControllers
    Warnings           = [string[]]$Warnings
}
```

## Task 1: Exact WPILOG parsing and brownout transitions

**Files:**

- Create: `tools/tests/Count-RobotBrownouts.Tests.ps1`
- Create: `tools/Count-RobotBrownouts.ps1`

- [ ] **Step 1: Write the dependency-free test harness and exact-count tests**

Create the test runner with assertion helpers, a WPILOG fixture writer that always uses 4-byte entry IDs, 4-byte payload sizes, and 8-byte timestamps, and these four tests:

```powershell
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$script:Passed = 0
$script:Failed = 0

function Invoke-Test {
    param([string]$Name, [scriptblock]$Body)
    try {
        & $Body
        $script:Passed++
        Write-Host "PASS $Name"
    } catch {
        $script:Failed++
        Write-Host "FAIL $Name"
        Write-Host "  $($_.Exception.Message)"
    }
}

function Assert-Equal {
    param($Expected, $Actual, [string]$Because = '')
    if ($Expected -ne $Actual) {
        throw "Expected <$Expected>, got <$Actual>. $Because"
    }
}

function Assert-True {
    param([bool]$Condition, [string]$Because = '')
    if (-not $Condition) {
        throw "Expected condition to be true. $Because"
    }
}

function Assert-Throws {
    param([scriptblock]$Body, [string]$Pattern)
    try {
        & $Body
    } catch {
        if ($_.Exception.Message -notmatch $Pattern) {
            throw "Expected error matching <$Pattern>, got <$($_.Exception.Message)>."
        }
        return
    }
    throw "Expected an exception matching <$Pattern>."
}

function Remove-TestDirectory {
    param([string]$LiteralPath)
    $full = [IO.Path]::GetFullPath($LiteralPath)
    $temporaryRoot = [IO.Path]::GetFullPath([IO.Path]::GetTempPath())
    if (-not $full.StartsWith(
            $temporaryRoot, [StringComparison]::OrdinalIgnoreCase
        ) -or
        -not [IO.Path]::GetFileName($full).StartsWith('FRCBrownoutTests-')) {
        throw "Refusing to remove unexpected test directory: $full"
    }
    if (Test-Path -LiteralPath $full) {
        Remove-Item -LiteralPath $full -Recurse -Force
    }
}

function Write-TestRecord {
    param(
        [System.IO.BinaryWriter]$Writer,
        [uint32]$Entry,
        [uint64]$Timestamp,
        [byte[]]$Payload
    )
    $Writer.Write([byte]0x7f)
    $Writer.Write($Entry)
    $Writer.Write([uint32]$Payload.Length)
    $Writer.Write($Timestamp)
    $Writer.Write($Payload)
}

function ConvertTo-TestStartPayload {
    param([uint32]$Entry, [string]$Name, [string]$Type)
    $stream = New-Object System.IO.MemoryStream
    $writer = New-Object System.IO.BinaryWriter($stream, [Text.Encoding]::UTF8, $true)
    try {
        $nameBytes = [Text.Encoding]::UTF8.GetBytes($Name)
        $typeBytes = [Text.Encoding]::UTF8.GetBytes($Type)
        $writer.Write([byte]0)
        $writer.Write($Entry)
        $writer.Write([uint32]$nameBytes.Length)
        $writer.Write($nameBytes)
        $writer.Write([uint32]$typeBytes.Length)
        $writer.Write($typeBytes)
        $writer.Write([uint32]0)
        $writer.Flush()
        return $stream.ToArray()
    } finally {
        $writer.Dispose()
        $stream.Dispose()
    }
}

function ConvertTo-TestValuePayload {
    param([string]$Type, $Value)
    switch ($Type) {
        'boolean' { return [byte[]]@([byte][bool]$Value) }
        'double' { return [BitConverter]::GetBytes([double]$Value) }
        'string' { return [Text.Encoding]::UTF8.GetBytes([string]$Value) }
        default { throw "Unsupported test type: $Type" }
    }
}

function New-TestWpiLog {
    param(
        [string]$LiteralPath,
        [object[]]$Signals,
        [switch]$TruncatedTail
    )
    $stream = [IO.File]::Open($LiteralPath, [IO.FileMode]::Create, [IO.FileAccess]::Write)
    $writer = New-Object IO.BinaryWriter($stream, [Text.Encoding]::UTF8, $true)
    try {
        $writer.Write([Text.Encoding]::ASCII.GetBytes('WPILOG'))
        $writer.Write([uint16]0x0100)
        $writer.Write([uint32]0)
        [uint32]$entry = 1
        foreach ($signal in $Signals) {
            Write-TestRecord $writer 0 0 (
                ConvertTo-TestStartPayload $entry $signal.Name $signal.Type
            )
            foreach ($sample in $signal.Samples) {
                Write-TestRecord $writer $entry ([uint64]$sample.Timestamp) (
                    ConvertTo-TestValuePayload $signal.Type $sample.Value
                )
            }
            $entry++
        }
        if ($TruncatedTail) {
            $writer.Write([byte]0x7f)
            $writer.Write([byte[]]@(1, 0))
        }
    } finally {
        $writer.Dispose()
        $stream.Dispose()
    }
}

$productionScript = Join-Path $PSScriptRoot '..\Count-RobotBrownouts.ps1'
if (Test-Path -LiteralPath $productionScript) {
    . $productionScript
}
$testRoot = Join-Path ([IO.Path]::GetTempPath()) (
    'FRCBrownoutTests-' + [guid]::NewGuid().ToString('N')
)
[void](New-Item -ItemType Directory -Path $testRoot)

try {
    Invoke-Test 'counts only false-to-true transitions' {
        $file = Join-Path $testRoot 'exact.wpilog'
        New-TestWpiLog $file @(
            [pscustomobject]@{
                Name = '/SystemStats/BrownedOut'
                Type = 'boolean'
                Samples = @(
                    [pscustomobject]@{ Timestamp = 100; Value = $false }
                    [pscustomobject]@{ Timestamp = 200; Value = $true }
                    [pscustomobject]@{ Timestamp = 300; Value = $true }
                    [pscustomobject]@{ Timestamp = 400; Value = $false }
                    [pscustomobject]@{ Timestamp = 500; Value = $true }
                )
            }
        )
        $parsed = Read-WpiLogSignals $file
        $result = Get-ExactBrownoutAnalysis $parsed $file
        Assert-Equal 2 $result.Count
    }

    Invoke-Test 'counts an initially true state once' {
        $file = Join-Path $testRoot 'initial-true.wpilog'
        New-TestWpiLog $file @(
            [pscustomobject]@{
                Name = 'SystemStats/BrownedOut'
                Type = 'boolean'
                Samples = @(
                    [pscustomobject]@{ Timestamp = 100; Value = $true }
                    [pscustomobject]@{ Timestamp = 200; Value = $true }
                )
            }
        )
        $result = Get-ExactBrownoutAnalysis (Read-WpiLogSignals $file) $file
        Assert-Equal 1 $result.Count
    }

    Invoke-Test 'sorts samples by timestamp before transitions' {
        $file = Join-Path $testRoot 'out-of-order.wpilog'
        New-TestWpiLog $file @(
            [pscustomobject]@{
                Name = '/SystemStats/BrownedOut'
                Type = 'boolean'
                Samples = @(
                    [pscustomobject]@{ Timestamp = 300; Value = $false }
                    [pscustomobject]@{ Timestamp = 100; Value = $true }
                    [pscustomobject]@{ Timestamp = 200; Value = $false }
                )
            }
        )
        $result = Get-ExactBrownoutAnalysis (Read-WpiLogSignals $file) $file
        Assert-Equal 1 $result.Count
        Assert-Equal 100 $result.Events[0].Timestamp
    }

    Invoke-Test 'keeps usable records before a partial tail' {
        $file = Join-Path $testRoot 'truncated.wpilog'
        New-TestWpiLog $file @(
            [pscustomobject]@{
                Name = '/SystemStats/BrownedOut'
                Type = 'boolean'
                Samples = @(
                    [pscustomobject]@{ Timestamp = 100; Value = $false }
                    [pscustomobject]@{ Timestamp = 200; Value = $true }
                )
            }
        ) -TruncatedTail
        $parsed = Read-WpiLogSignals $file
        $result = Get-ExactBrownoutAnalysis $parsed $file
        Assert-True $parsed.TruncatedTail
        Assert-Equal 1 $result.Count
        Assert-True ($result.Warnings.Count -eq 1)
    }
} finally {
    Remove-TestDirectory $testRoot
}

Write-Host "$($script:Passed) passed, $($script:Failed) failed"
if ($script:Failed -ne 0) { exit 1 }
```

- [ ] **Step 2: Run the tests and verify the exact behavior is red**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File tools\tests\Count-RobotBrownouts.Tests.ps1
```

Expected: exit code 1 and failures stating that `Read-WpiLogSignals` or `Get-ExactBrownoutAnalysis` is not recognized.

- [ ] **Step 3: Implement the minimal WPILOG reader and exact analyzer**

Start `tools/Count-RobotBrownouts.ps1` with a non-mandatory parameter declaration so tests can dot-source it:

```powershell
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string[]]$Path = @(),
    [string]$OwletPath,
    [switch]$Detailed
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Read-VariableUInt {
    param([IO.BinaryReader]$Reader, [int]$ByteCount)
    $buffer = New-Object byte[] 8
    $read = $Reader.Read($buffer, 0, $ByteCount)
    if ($read -ne $ByteCount) {
        throw [IO.EndOfStreamException]::new('Incomplete record header.')
    }
    if (-not [BitConverter]::IsLittleEndian) {
        throw [System.PlatformNotSupportedException]::new(
            'The standalone Windows reader requires a little-endian runtime.'
        )
    }
    return [BitConverter]::ToUInt64($buffer, 0)
}

function Read-LengthPrefixedUtf8 {
    param([IO.BinaryReader]$Reader)
    [uint32]$length = $Reader.ReadUInt32()
    [int64]$remaining = $Reader.BaseStream.Length - $Reader.BaseStream.Position
    if ($length -gt $remaining -or $length -gt [int]::MaxValue) {
        throw [FormatException]::new('Invalid length-prefixed string.')
    }
    return [Text.Encoding]::UTF8.GetString($Reader.ReadBytes([int]$length))
}

function Read-WpiLogSignals {
    param([Parameter(Mandatory = $true)][string]$LiteralPath)

    $brown = New-Object 'Collections.Generic.List[object]'
    $modes = New-Object 'Collections.Generic.List[object]'
    $enables = New-Object 'Collections.Generic.List[object]'
    $faults = New-Object 'Collections.Generic.List[object]'
    $entries = @{}
    $hasBrown = $false
    $hasMode = $false
    $hasEnable = $false
    $hasFault = $false
    $truncated = $false
    [int64]$sequence = 0

    $stream = [IO.File]::Open(
        $LiteralPath, [IO.FileMode]::Open, [IO.FileAccess]::Read, [IO.FileShare]::Read
    )
    $reader = New-Object IO.BinaryReader($stream, [Text.Encoding]::UTF8, $true)
    try {
        if ($stream.Length -lt 12) { throw "Invalid WPILOG header: $LiteralPath" }
        $magic = [Text.Encoding]::ASCII.GetString($reader.ReadBytes(6))
        if ($magic -ne 'WPILOG') { throw "Invalid WPILOG header: $LiteralPath" }
        [uint16]$version = $reader.ReadUInt16()
        if (($version -shr 8) -ne 1) {
            throw "Unsupported WPILOG version 0x$($version.ToString('x4'))."
        }
        [uint32]$extraLength = $reader.ReadUInt32()
        if ($extraLength -gt ($stream.Length - $stream.Position)) {
            throw "Invalid WPILOG extra header length: $LiteralPath"
        }
        [void]$reader.ReadBytes([int]$extraLength)

        while ($stream.Position -lt $stream.Length) {
            try {
                [byte]$header = $reader.ReadByte()
                if (($header -band 0x80) -ne 0) {
                    throw [FormatException]::new('Record header spare bit is set.')
                }
                $entryBytes = ($header -band 0x03) + 1
                $sizeBytes = (($header -shr 2) -band 0x03) + 1
                $timestampBytes = (($header -shr 4) -band 0x07) + 1
                [uint64]$entry = Read-VariableUInt $reader $entryBytes
                [uint64]$payloadLength = Read-VariableUInt $reader $sizeBytes
                [uint64]$timestamp = Read-VariableUInt $reader $timestampBytes
                if ($payloadLength -gt [int]::MaxValue -or
                    $payloadLength -gt ($stream.Length - $stream.Position)) {
                    throw [IO.EndOfStreamException]::new('Incomplete record payload.')
                }

                $sequence++
                if ($entry -eq 0) {
                    $payload = $reader.ReadBytes([int]$payloadLength)
                    $payloadStream = New-Object IO.MemoryStream(,$payload)
                    $payloadReader = New-Object IO.BinaryReader(
                        $payloadStream, [Text.Encoding]::UTF8, $true
                    )
                    try {
                        [byte]$controlType = $payloadReader.ReadByte()
                        if ($controlType -eq 0) {
                            [uint32]$startedEntry = $payloadReader.ReadUInt32()
                            $name = Read-LengthPrefixedUtf8 $payloadReader
                            $type = Read-LengthPrefixedUtf8 $payloadReader
                            [void](Read-LengthPrefixedUtf8 $payloadReader)
                            $normalized = $name.TrimStart('/')
                            $kind = $null
                            if ($normalized -eq 'SystemStats/BrownedOut') {
                                $kind = 'brown'; $hasBrown = $true
                            } elseif ($normalized -eq 'RobotMode') {
                                $kind = 'mode'; $hasMode = $true
                            } elseif ($normalized -eq 'RobotEnable') {
                                $kind = 'enable'; $hasEnable = $true
                            } elseif ($normalized.EndsWith('/Fault_BridgeBrownout')) {
                                $kind = 'fault'; $hasFault = $true
                            }
                            $entries[[uint64]$startedEntry] = [pscustomobject]@{
                                Name = $name; Type = $type; Kind = $kind
                            }
                        } elseif ($controlType -eq 1) {
                            [uint32]$finishedEntry = $payloadReader.ReadUInt32()
                            $entries.Remove([uint64]$finishedEntry)
                        }
                    } finally {
                        $payloadReader.Dispose()
                        $payloadStream.Dispose()
                    }
                    continue
                }

                $definition = $entries[[uint64]$entry]
                if ($null -eq $definition -or $null -eq $definition.Kind) {
                    [void]$stream.Seek([int64]$payloadLength, [IO.SeekOrigin]::Current)
                    continue
                }
                $data = $reader.ReadBytes([int]$payloadLength)
                switch ($definition.Type) {
                    'boolean' { $value = $data.Length -gt 0 -and $data[0] -ne 0 }
                    'double' {
                        if ($data.Length -ne 8) { throw 'Invalid double payload.' }
                        $value = [BitConverter]::ToDouble($data, 0)
                    }
                    'string' { $value = [Text.Encoding]::UTF8.GetString($data) }
                    default { continue }
                }
                $sample = [pscustomobject]@{
                    Timestamp = [int64]$timestamp
                    Sequence = $sequence
                    Value = $value
                    Name = $definition.Name
                    Source = $LiteralPath
                }
                switch ($definition.Kind) {
                    'brown' { $brown.Add($sample) }
                    'mode' { $modes.Add($sample) }
                    'enable' { $enables.Add($sample) }
                    'fault' { $faults.Add($sample) }
                }
            } catch [IO.EndOfStreamException] {
                $truncated = $true
                break
            }
        }
    } finally {
        $reader.Dispose()
        $stream.Dispose()
    }

    return [pscustomobject]@{
        Source = $LiteralPath
        BrownedOut = @($brown)
        RobotMode = @($modes)
        RobotEnable = @($enables)
        BridgeFault = @($faults)
        HasBrownedOutEntry = $hasBrown
        HasRobotModeEntry = $hasMode
        HasRobotEnableEntry = $hasEnable
        HasBridgeFaultEntry = $hasFault
        TruncatedTail = $truncated
    }
}

function Get-ExactBrownoutAnalysis {
    param($Signals, [string]$Source)
    if (-not $Signals.HasBrownedOutEntry) {
        throw "No SystemStats/BrownedOut entry found in $Source"
    }
    $events = New-Object 'Collections.Generic.List[object]'
    $hasPrevious = $false
    $previous = $false
    foreach ($sample in @($Signals.BrownedOut | Sort-Object Timestamp, Sequence)) {
        $current = [bool]$sample.Value
        if ($current -and (-not $hasPrevious -or -not $previous)) {
            $events.Add($sample)
        }
        $previous = $current
        $hasPrevious = $true
    }
    $warnings = @()
    if ($Signals.TruncatedTail) {
        $warnings += 'The WPILOG has a partial final record; preceding data was used.'
    }
    return [pscustomobject]@{
        Source = $Source
        Count = $events.Count
        Confidence = 'exact'
        Method = 'ExactRoboRioBrownedOut'
        Signal = 'SystemStats/BrownedOut'
        Events = @($events)
        FaultControllers = @()
        Warnings = $warnings
    }
}
```

- [ ] **Step 4: Run the exact-count tests and verify green**

Run the test command from Step 2.

Expected: `4 passed, 0 failed`, exit code 0.

- [ ] **Step 5: Commit the exact WPILOG slice**

```powershell
git add -- tools\Count-RobotBrownouts.ps1 tools\tests\Count-RobotBrownouts.Tests.ps1
git commit -m "feat: count exact WPILOG brownouts"
```

## Task 2: Hoot enable-interruption estimates and paired-bus deduplication

**Files:**

- Modify: `tools/tests/Count-RobotBrownouts.Tests.ps1`
- Modify: `tools/Count-RobotBrownouts.ps1`

- [ ] **Step 1: Add failing Hoot semantic tests**

Before the test runner's `finally`, add helpers and four tests:

```powershell
function New-TestSample {
    param([int64]$Timestamp, $Value, [string]$Name, [string]$Source = 'test')
    return [pscustomobject]@{
        Timestamp = $Timestamp
        Sequence = $Timestamp
        Value = $Value
        Name = $Name
        Source = $Source
    }
}

function New-TestParsedSignals {
    param(
        [object[]]$Modes = @(),
        [object[]]$Enables = @(),
        [object[]]$Faults = @(),
        [string]$Source = 'test'
    )
    return [pscustomobject]@{
        Source = $Source
        BrownedOut = @()
        RobotMode = $Modes
        RobotEnable = $Enables
        BridgeFault = $Faults
        HasBrownedOutEntry = $false
        HasRobotModeEntry = $Modes.Count -gt 0
        HasRobotEnableEntry = $Enables.Count -gt 0
        HasBridgeFaultEntry = $Faults.Count -gt 0
        TruncatedTail = $false
    }
}

Invoke-Test 'counts only bounded returns to the same enabled mode' {
    $signals = New-TestParsedSignals -Modes @(
        (New-TestSample 0 'Disabled' 'RobotMode')
        (New-TestSample 1000000 'Autonomous' 'RobotMode')
        (New-TestSample 2000000 'Disabled' 'RobotMode')
        (New-TestSample 2080000 'Autonomous' 'RobotMode')
        (New-TestSample 3000000 'Disabled' 'RobotMode')
        (New-TestSample 5500000 'Teleop' 'RobotMode')
        (New-TestSample 6000000 'Disabled' 'RobotMode')
        (New-TestSample 6090000 'Teleop' 'RobotMode')
        (New-TestSample 7000000 'Disabled' 'RobotMode')
    )
    $result = Get-HootBrownoutAnalysis @($signals) 'mode-test'
    Assert-Equal 2 $result.Count
    Assert-Equal 'RobotMode' $result.Signal
}

Invoke-Test 'falls back to bounded RobotEnable interruptions' {
    $signals = New-TestParsedSignals -Enables @(
        (New-TestSample 0 $false 'RobotEnable')
        (New-TestSample 1000000 $true 'RobotEnable')
        (New-TestSample 2000000 $false 'RobotEnable')
        (New-TestSample 2100000 $true 'RobotEnable')
        (New-TestSample 4000000 $false 'RobotEnable')
    )
    $result = Get-HootBrownoutAnalysis @($signals) 'enable-test'
    Assert-Equal 1 $result.Count
    Assert-Equal 'RobotEnable' $result.Signal
}

Invoke-Test 'deduplicates paired bus events within 25 milliseconds' {
    $first = New-TestParsedSignals -Source 'rio' -Modes @(
        (New-TestSample 0 'Teleop' 'RobotMode' 'rio')
        (New-TestSample 1000000 'Disabled' 'RobotMode' 'rio')
        (New-TestSample 1080000 'Teleop' 'RobotMode' 'rio')
    )
    $second = New-TestParsedSignals -Source 'canivore' -Modes @(
        (New-TestSample 0 'Teleop' 'RobotMode' 'canivore')
        (New-TestSample 1009000 'Disabled' 'RobotMode' 'canivore')
        (New-TestSample 1089000 'Teleop' 'RobotMode' 'canivore')
    )
    $result = Get-HootBrownoutAnalysis @($first, $second) 'paired'
    Assert-Equal 1 $result.Count
}

Invoke-Test 'reports bridge faults as evidence without increasing count' {
    $signals = New-TestParsedSignals -Modes @(
        (New-TestSample 0 'Teleop' 'RobotMode')
        (New-TestSample 1000000 'Disabled' 'RobotMode')
        (New-TestSample 1080000 'Teleop' 'RobotMode')
    ) -Faults @(
        (New-TestSample 900000 0.0 'Phoenix6/TalonFX-3/Fault_BridgeBrownout')
        (New-TestSample 1010000 1.0 'Phoenix6/TalonFX-3/Fault_BridgeBrownout')
        (New-TestSample 1020000 1.0 'Phoenix6/TalonFX-3/Fault_BridgeBrownout')
        (New-TestSample 1030000 1.0 'Phoenix6/TalonFX-4/Fault_BridgeBrownout')
    )
    $result = Get-HootBrownoutAnalysis @($signals) 'fault-evidence'
    Assert-Equal 1 $result.Count
    Assert-Equal 2 $result.FaultControllers.Count
}

Invoke-Test 'parses converted Hoot mode and bridge-fault entries' {
    $file = Join-Path $testRoot 'converted-hoot.wpilog'
    New-TestWpiLog $file @(
        [pscustomobject]@{
            Name = 'RobotMode'
            Type = 'string'
            Samples = @(
                [pscustomobject]@{ Timestamp = 0; Value = 'Teleop' }
                [pscustomobject]@{ Timestamp = 1000000; Value = 'Disabled' }
                [pscustomobject]@{ Timestamp = 1080000; Value = 'Teleop' }
            )
        }
        [pscustomobject]@{
            Name = 'Phoenix6/TalonFX-3/Fault_BridgeBrownout'
            Type = 'double'
            Samples = @(
                [pscustomobject]@{ Timestamp = 900000; Value = 0.0 }
                [pscustomobject]@{ Timestamp = 1010000; Value = 1.0 }
            )
        }
    )
    $signals = Read-WpiLogSignals $file
    $result = Get-HootBrownoutAnalysis @($signals) $file
    Assert-Equal 1 $result.Count
    Assert-Equal 1 $result.FaultControllers.Count
}
```

- [ ] **Step 2: Run tests and verify the new Hoot tests fail**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File tools\tests\Count-RobotBrownouts.Tests.ps1
```

Expected: the original 4 tests pass and the new tests fail because `Get-HootBrownoutAnalysis` is not recognized.

- [ ] **Step 3: Implement bounded Hoot events, evidence, and merge**

Append these functions:

```powershell
function Get-HootEventsFromSignals {
    param($Signals)
    $events = New-Object 'Collections.Generic.List[object]'
    $signalUsed = $null

    if ($Signals.HasRobotModeEntry) {
        $signalUsed = 'RobotMode'
        $previousMode = $null
        $pending = $null
        foreach ($sample in @($Signals.RobotMode | Sort-Object Timestamp, Sequence)) {
            $mode = [string]$sample.Value
            if ($mode -eq $previousMode) { continue }
            $enabled = $mode -in @('Autonomous', 'Teleop', 'Test')
            $previousEnabled = $previousMode -in @('Autonomous', 'Teleop', 'Test')
            if ($mode -eq 'Disabled' -and $previousEnabled) {
                $pending = [pscustomobject]@{
                    Timestamp = [int64]$sample.Timestamp
                    FromMode = $previousMode
                    Source = $Signals.Source
                }
            } elseif ($enabled) {
                if ($null -ne $pending -and
                    $mode -eq $pending.FromMode -and
                    ($sample.Timestamp - $pending.Timestamp) -le 1000000) {
                    $events.Add([pscustomobject]@{
                        Timestamp = $pending.Timestamp
                        EndTimestamp = [int64]$sample.Timestamp
                        Source = $Signals.Source
                        Signal = 'RobotMode'
                    })
                }
                $pending = $null
            } else {
                $pending = $null
            }
            $previousMode = $mode
        }
    } elseif ($Signals.HasRobotEnableEntry) {
        $signalUsed = 'RobotEnable'
        $hasPrevious = $false
        $previous = $false
        $pending = $null
        foreach ($sample in @($Signals.RobotEnable | Sort-Object Timestamp, Sequence)) {
            $current = [bool]$sample.Value
            if ($hasPrevious -and $previous -and -not $current) {
                $pending = [pscustomobject]@{
                    Timestamp = [int64]$sample.Timestamp
                    Source = $Signals.Source
                }
            } elseif ($current -and -not $previous) {
                if ($null -ne $pending -and
                    ($sample.Timestamp - $pending.Timestamp) -le 1000000) {
                    $events.Add([pscustomobject]@{
                        Timestamp = $pending.Timestamp
                        EndTimestamp = [int64]$sample.Timestamp
                        Source = $Signals.Source
                        Signal = 'RobotEnable'
                    })
                }
                $pending = $null
            }
            $previous = $current
            $hasPrevious = $true
        }
    }

    return [pscustomobject]@{
        Signal = $signalUsed
        Events = @($events)
    }
}

function Merge-HootEvents {
    param([object[]]$Events)
    $merged = New-Object 'Collections.Generic.List[object]'
    foreach ($event in @($Events | Sort-Object Timestamp)) {
        if ($merged.Count -eq 0 -or
            ($event.Timestamp - $merged[$merged.Count - 1].Timestamp) -gt 25000) {
            $merged.Add([pscustomobject]@{
                Timestamp = [int64]$event.Timestamp
                EndTimestamp = [int64]$event.EndTimestamp
                Sources = @($event.Source)
            })
        } else {
            $current = $merged[$merged.Count - 1]
            $current.Sources = @($current.Sources + $event.Source | Select-Object -Unique)
            if ($event.EndTimestamp -gt $current.EndTimestamp) {
                $current.EndTimestamp = [int64]$event.EndTimestamp
            }
        }
    }
    return @($merged)
}

function Get-HootBrownoutAnalysis {
    param([object[]]$SignalSets, [string]$Source)
    $allEvents = @()
    $signalsUsed = @()
    $controllers = New-Object 'Collections.Generic.HashSet[string]' (
        [StringComparer]::OrdinalIgnoreCase
    )
    $warnings = @()

    foreach ($signals in $SignalSets) {
        $derived = Get-HootEventsFromSignals $signals
        if ($null -ne $derived.Signal) { $signalsUsed += $derived.Signal }
        $allEvents += $derived.Events
        $faultState = @{}
        foreach ($sample in @($signals.BridgeFault | Sort-Object Timestamp, Sequence)) {
            $current = [double]$sample.Value -gt 0.5
            $previous = $faultState[$sample.Name]
            if ($current -and $previous -ne $true) {
                $controller = $sample.Name.Substring(
                    0, $sample.Name.Length - '/Fault_BridgeBrownout'.Length
                )
                [void]$controllers.Add($controller)
            }
            $faultState[$sample.Name] = $current
        }
        if ($signals.TruncatedTail) {
            $warnings += "The converted log $($signals.Source) has a partial final record."
        }
    }

    if ($signalsUsed.Count -eq 0) {
        throw "No global RobotMode or RobotEnable signal found in Hoot session $Source"
    }
    $events = @(Merge-HootEvents $allEvents)
    return [pscustomobject]@{
        Source = $Source
        Count = $events.Count
        Confidence = 'estimated'
        Method = 'EstimatedHootEnableInterruptions'
        Signal = (@($signalsUsed | Select-Object -Unique) -join ',')
        Events = $events
        FaultControllers = @($controllers | Sort-Object)
        Warnings = @($warnings | Select-Object -Unique)
    }
}
```

- [ ] **Step 4: Run all semantic tests and verify green**

Run the test command from Step 2.

Expected: `9 passed, 0 failed`.

- [ ] **Step 5: Commit the Hoot semantic slice**

```powershell
git add -- tools\Count-RobotBrownouts.ps1 tools\tests\Count-RobotBrownouts.Tests.ps1
git commit -m "feat: estimate Hoot brownout interruptions"
```

## Task 3: File discovery, sibling suppression, and Hoot session grouping

**Files:**

- Modify: `tools/tests/Count-RobotBrownouts.Tests.ps1`
- Modify: `tools/Count-RobotBrownouts.ps1`

- [ ] **Step 1: Add failing discovery and grouping tests**

Add tests that create nested one-byte files under `$testRoot`:

```powershell
Invoke-Test 'groups same-timestamp raw Hoot buses into one session' {
    $folder = Join-Path $testRoot 'grouping'
    [void](New-Item -ItemType Directory -Path $folder)
    $rio = Join-Path $folder 'MATCH_rio_2026-06-27_21-02-40.hoot'
    $canivore = Join-Path $folder 'MATCH_SERIAL_2026-06-27_21-02-40.hoot'
    [IO.File]::WriteAllBytes($rio, [byte[]]@(1))
    [IO.File]::WriteAllBytes($canivore, [byte[]]@(2))
    $inventory = Get-InputInventory @($folder)
    Assert-Equal 1 $inventory.HootSessions.Count
    Assert-Equal 2 $inventory.HootSessions[0].Paths.Count
}

Invoke-Test 'suppresses a converted sibling during directory discovery' {
    $folder = Join-Path $testRoot 'sibling'
    [void](New-Item -ItemType Directory -Path $folder)
    $raw = Join-Path $folder 'MATCH_rio_2026-06-27_21-02-40.hoot'
    $converted = [IO.Path]::ChangeExtension($raw, '.wpilog')
    [IO.File]::WriteAllBytes($raw, [byte[]]@(1))
    [IO.File]::WriteAllBytes($converted, [byte[]]@(2))
    $inventory = Get-InputInventory @($folder)
    Assert-Equal 0 $inventory.WpiLogs.Count
    Assert-Equal 1 $inventory.HootSessions.Count
}

Invoke-Test 'keeps an explicitly requested converted WPILOG' {
    $folder = Join-Path $testRoot 'explicit'
    [void](New-Item -ItemType Directory -Path $folder)
    $raw = Join-Path $folder 'MATCH_rio_2026-06-27_21-02-40.hoot'
    $converted = [IO.Path]::ChangeExtension($raw, '.wpilog')
    [IO.File]::WriteAllBytes($raw, [byte[]]@(1))
    [IO.File]::WriteAllBytes($converted, [byte[]]@(2))
    $inventory = Get-InputInventory @($converted)
    Assert-Equal 1 $inventory.WpiLogs.Count
    Assert-Equal 0 $inventory.HootSessions.Count
}

Invoke-Test 'rejects a requested path with no supported files' {
    $folder = Join-Path $testRoot 'empty'
    [void](New-Item -ItemType Directory -Path $folder)
    Assert-Throws { Get-InputInventory @($folder) } 'no supported'
}
```

- [ ] **Step 2: Run tests and verify discovery tests fail**

Run the full test command.

Expected: the first 9 tests pass and the 4 discovery tests fail because `Get-InputInventory` is not recognized.

- [ ] **Step 3: Implement deterministic recursive discovery and grouping**

Append:

```powershell
function Get-InputInventory {
    param([string[]]$RequestedPaths)
    $records = New-Object 'Collections.Generic.List[object]'
    $seen = New-Object 'Collections.Generic.HashSet[string]' (
        [StringComparer]::OrdinalIgnoreCase
    )

    foreach ($requested in $RequestedPaths) {
        if (-not (Test-Path -LiteralPath $requested)) {
            throw "Input path does not exist: $requested"
        }
        $item = Get-Item -LiteralPath $requested
        $files = if ($item.PSIsContainer) {
            @(Get-ChildItem -LiteralPath $item.FullName -Recurse -File |
                Where-Object { $_.Extension -in @('.wpilog', '.hoot') })
        } else {
            if ($item.Extension -notin @('.wpilog', '.hoot')) {
                throw "Unsupported input extension: $($item.FullName)"
            }
            @($item)
        }
        foreach ($file in $files) {
            if ($seen.Add($file.FullName)) {
                $records.Add([pscustomobject]@{
                    Path = $file.FullName
                    Extension = $file.Extension.ToLowerInvariant()
                    Explicit = -not $item.PSIsContainer
                })
            }
        }
    }

    if ($records.Count -eq 0) {
        throw 'The requested path contains no supported .wpilog or .hoot files.'
    }

    $rawBases = New-Object 'Collections.Generic.HashSet[string]' (
        [StringComparer]::OrdinalIgnoreCase
    )
    foreach ($record in @($records | Where-Object Extension -eq '.hoot')) {
        [void]$rawBases.Add([IO.Path]::ChangeExtension($record.Path, $null))
    }
    $wpiLogs = @($records | Where-Object {
        $_.Extension -eq '.wpilog' -and (
            $_.Explicit -or
            -not $rawBases.Contains([IO.Path]::ChangeExtension($_.Path, $null))
        )
    } | ForEach-Object Path)

    $groups = @{}
    foreach ($record in @($records | Where-Object Extension -eq '.hoot')) {
        $baseName = [IO.Path]::GetFileNameWithoutExtension($record.Path)
        $directory = [IO.Path]::GetDirectoryName($record.Path)
        if ($baseName -match '(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})$') {
            $key = "$directory|$($Matches[1])"
            $label = "$directory [$($Matches[1])]"
        } else {
            $key = $record.Path
            $label = $record.Path
        }
        if (-not $groups.ContainsKey($key)) {
            $groups[$key] = [pscustomobject]@{
                Key = $key
                Source = $label
                Paths = New-Object 'Collections.Generic.List[string]'
            }
        }
        $groups[$key].Paths.Add($record.Path)
    }

    return [pscustomobject]@{
        WpiLogs = @($wpiLogs | Sort-Object -Unique)
        HootSessions = @($groups.Values | Sort-Object Source | ForEach-Object {
            [pscustomobject]@{
                Key = $_.Key
                Source = $_.Source
                Paths = @($_.Paths | Sort-Object -Unique)
            }
        })
    }
}
```

- [ ] **Step 4: Run all tests and verify grouping green**

Run the full test command.

Expected: `13 passed, 0 failed`.

- [ ] **Step 5: Commit discovery**

```powershell
git add -- tools\Count-RobotBrownouts.ps1 tools\tests\Count-RobotBrownouts.Tests.ps1
git commit -m "feat: group multi-bus Hoot sessions"
```

## Task 4: Owlet resolution, checksum enforcement, and conversion

**Files:**

- Modify: `tools/tests/Count-RobotBrownouts.Tests.ps1`
- Modify: `tools/Count-RobotBrownouts.ps1`

- [ ] **Step 1: Add failing offline Owlet tests**

Add these tests. They use fake executables and a synthetic converted WPILOG; they do not use the network:

```powershell
Invoke-Test 'uses an explicit Owlet executable' {
    $fake = Join-Path $testRoot 'explicit-owlet.cmd'
    [IO.File]::WriteAllText($fake, '@exit /b 0', [Text.Encoding]::ASCII)
    $resolved = Resolve-Owlet -RequestedPath $fake -CacheDirectory $testRoot
    Assert-Equal ([IO.Path]::GetFullPath($fake)) $resolved
}

Invoke-Test 'accepts a cache entry with the expected SHA-1' {
    $cache = Join-Path $testRoot 'cache-hit'
    [void](New-Item -ItemType Directory -Path $cache)
    $fake = Join-Path $cache 'owlet-26.3.0-windowsx86-64.exe'
    [IO.File]::WriteAllText($fake, 'fake owlet', [Text.Encoding]::ASCII)
    $hash = Get-FileSha1 $fake
    $resolved = Resolve-Owlet -CacheDirectory $cache -ExpectedSha1 $hash
    Assert-Equal $fake $resolved
}

Invoke-Test 'deletes and rejects a corrupt cached Owlet' {
    $cache = Join-Path $testRoot 'cache-bad'
    [void](New-Item -ItemType Directory -Path $cache)
    $fake = Join-Path $cache 'owlet-26.3.0-windowsx86-64.exe'
    [IO.File]::WriteAllText($fake, 'corrupt', [Text.Encoding]::ASCII)
    Assert-Throws {
        Resolve-Owlet -CacheDirectory $cache -ExpectedSha1 ('0' * 40)
    } 'checksum'
    Assert-True (-not (Test-Path -LiteralPath $fake))
}

Invoke-Test 'keeps usable conversion output after a nonzero Owlet exit' {
    $converted = Join-Path $testRoot 'fake-converted.wpilog'
    New-TestWpiLog $converted @(
        [pscustomobject]@{
            Name = 'RobotMode'
            Type = 'string'
            Samples = @(
                [pscustomobject]@{ Timestamp = 0; Value = 'Teleop' }
                [pscustomobject]@{ Timestamp = 1000000; Value = 'Disabled' }
                [pscustomobject]@{ Timestamp = 1080000; Value = 'Teleop' }
            )
        }
    )
    $fake = Join-Path $testRoot 'fake-converter.cmd'
    [IO.File]::WriteAllText(
        $fake,
        "@echo off`r`ncopy /Y `"$converted`" `"%~4`" >nul`r`nexit /b 7`r`n",
        [Text.Encoding]::ASCII
    )
    $raw = Join-Path $testRoot 'input.hoot'
    [IO.File]::WriteAllBytes($raw, [byte[]]@(1))
    $output = Join-Path $testRoot 'output.wpilog'
    $conversion = Invoke-OwletConversion $fake $raw $output
    Assert-True (Test-Path -LiteralPath $output)
    Assert-True ($conversion.Warning -match 'code 7')
}
```

- [ ] **Step 2: Run tests and verify Owlet tests fail**

Run the full test command.

Expected: 13 tests pass and 4 tests fail because the Owlet functions are not recognized.

- [ ] **Step 3: Implement pinned acquisition and resilient conversion**

Append constants and functions:

```powershell
$script:OwletVersion = '26.3.0'
$script:OwletFileName = 'owlet-26.3.0-windowsx86-64.exe'
$script:OwletUrl =
    'https://redist.ctr-electronics.com/tools/owlet/26.3.0/' +
    $script:OwletFileName
$script:OwletSha1 = '01df8c1aa20cb0f6af47d550c860474a30c6be36'

function Get-FileSha1 {
    param([string]$LiteralPath)
    return (Get-FileHash -LiteralPath $LiteralPath -Algorithm SHA1).Hash.ToLowerInvariant()
}

function Test-WpiLogHeader {
    param([string]$LiteralPath)
    if (-not (Test-Path -LiteralPath $LiteralPath)) { return $false }
    $stream = [IO.File]::OpenRead($LiteralPath)
    try {
        if ($stream.Length -lt 12) { return $false }
        $buffer = New-Object byte[] 6
        if ($stream.Read($buffer, 0, 6) -ne 6) { return $false }
        return [Text.Encoding]::ASCII.GetString($buffer) -eq 'WPILOG'
    } finally {
        $stream.Dispose()
    }
}

function Resolve-Owlet {
    param(
        [string]$RequestedPath,
        [string]$CacheDirectory = (Join-Path $env:LOCALAPPDATA 'FRCBrownoutCounter'),
        [string]$ExpectedSha1 = $script:OwletSha1
    )
    if ($RequestedPath) {
        if (-not (Test-Path -LiteralPath $RequestedPath -PathType Leaf)) {
            throw "Owlet executable does not exist: $RequestedPath"
        }
        return [IO.Path]::GetFullPath($RequestedPath)
    }

    $onPath = Get-Command owlet, owlet.exe -CommandType Application `
        -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($null -ne $onPath) { return $onPath.Source }

    if ([string]::IsNullOrWhiteSpace($CacheDirectory)) {
        $CacheDirectory = Join-Path $HOME '.frc-brownout-counter'
    }
    $cached = Join-Path $CacheDirectory $script:OwletFileName
    if (Test-Path -LiteralPath $cached) {
        if ((Get-FileSha1 $cached) -ne $ExpectedSha1.ToLowerInvariant()) {
            Remove-Item -LiteralPath $cached -Force
            throw 'Cached Owlet checksum does not match CTRE''s published checksum.'
        }
        return $cached
    }

    [void](New-Item -ItemType Directory -Path $CacheDirectory -Force)
    $partial = "$cached.partial"
    try {
        [Net.ServicePointManager]::SecurityProtocol =
            [Net.ServicePointManager]::SecurityProtocol -bor
            [Net.SecurityProtocolType]::Tls12
        Invoke-WebRequest -Uri $script:OwletUrl -OutFile $partial -UseBasicParsing
        if ((Get-FileSha1 $partial) -ne $ExpectedSha1.ToLowerInvariant()) {
            throw 'Downloaded Owlet checksum does not match CTRE''s published checksum.'
        }
        Move-Item -LiteralPath $partial -Destination $cached
        return $cached
    } catch {
        if (Test-Path -LiteralPath $partial) {
            Remove-Item -LiteralPath $partial -Force
        }
        throw
    }
}

function Invoke-OwletConversion {
    param(
        [string]$Executable,
        [string]$InputHoot,
        [string]$OutputWpiLog
    )
    $nativeOutput = @(& $Executable -f wpilog $InputHoot $OutputWpiLog 2>&1)
    $exitCode = $LASTEXITCODE
    $valid = Test-WpiLogHeader $OutputWpiLog
    if (-not $valid) {
        $message = @($nativeOutput | ForEach-Object ToString) -join [Environment]::NewLine
        throw "Owlet conversion failed for $InputHoot (code $exitCode). $message"
    }
    $warning = $null
    if ($exitCode -ne 0) {
        $warning =
            "Owlet returned code $exitCode for $InputHoot; usable preceding data was retained."
    }
    return [pscustomobject]@{
        OutputPath = $OutputWpiLog
        ExitCode = $exitCode
        Warning = $warning
        NativeOutput = @($nativeOutput | ForEach-Object ToString)
    }
}
```

- [ ] **Step 4: Run all unit tests and verify green**

Run the full test command.

Expected: `17 passed, 0 failed`.

- [ ] **Step 5: Commit Owlet support**

```powershell
git add -- tools\Count-RobotBrownouts.ps1 tools\tests\Count-RobotBrownouts.Tests.ps1
git commit -m "feat: decode raw Hoot logs with Owlet"
```

## Task 5: End-to-end orchestration, cleanup, and CLI output

**Files:**

- Modify: `tools/tests/Count-RobotBrownouts.Tests.ps1`
- Modify: `tools/Count-RobotBrownouts.ps1`

- [ ] **Step 1: Add failing CLI tests**

Add a subprocess helper and these tests:

```powershell
function Invoke-CounterProcess {
    param([string[]]$Arguments)
    $outputFile = Join-Path $testRoot ([guid]::NewGuid().ToString('N') + '.out')
    $errorFile = Join-Path $testRoot ([guid]::NewGuid().ToString('N') + '.err')
    $process = Start-Process -FilePath 'powershell.exe' -ArgumentList (
        @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', "`"$productionScript`"") +
        $Arguments
    ) -Wait -PassThru -RedirectStandardOutput $outputFile `
      -RedirectStandardError $errorFile -WindowStyle Hidden
    return [pscustomobject]@{
        ExitCode = $process.ExitCode
        Output = [IO.File]::ReadAllText($outputFile)
        Error = [IO.File]::ReadAllText($errorFile)
    }
}

Invoke-Test 'CLI prints one exact result' {
    $file = Join-Path $testRoot 'cli-exact.wpilog'
    New-TestWpiLog $file @(
        [pscustomobject]@{
            Name = '/SystemStats/BrownedOut'
            Type = 'boolean'
            Samples = @(
                [pscustomobject]@{ Timestamp = 0; Value = $false }
                [pscustomobject]@{ Timestamp = 1000000; Value = $true }
            )
        }
    )
    $run = Invoke-CounterProcess @("`"$file`"")
    Assert-Equal 0 $run.ExitCode
    Assert-True ($run.Output -match 'Brownouts: 1 \(exact\)')
}

Invoke-Test 'CLI combines two raw Hoot inputs into one estimated result' {
    $converted = Join-Path $testRoot 'cli-converted.wpilog'
    New-TestWpiLog $converted @(
        [pscustomobject]@{
            Name = 'RobotMode'
            Type = 'string'
            Samples = @(
                [pscustomobject]@{ Timestamp = 0; Value = 'Teleop' }
                [pscustomobject]@{ Timestamp = 1000000; Value = 'Disabled' }
                [pscustomobject]@{ Timestamp = 1080000; Value = 'Teleop' }
            )
        }
    )
    $fake = Join-Path $testRoot 'cli-owlet.cmd'
    [IO.File]::WriteAllText(
        $fake,
        "@echo off`r`ncopy /Y `"$converted`" `"%~4`" >nul`r`nexit /b 0`r`n",
        [Text.Encoding]::ASCII
    )
    $folder = Join-Path $testRoot 'cli-hoot'
    [void](New-Item -ItemType Directory -Path $folder)
    [IO.File]::WriteAllBytes(
        (Join-Path $folder 'M_rio_2026-01-01_00-00-00.hoot'), [byte[]]@(1)
    )
    [IO.File]::WriteAllBytes(
        (Join-Path $folder 'M_canivore_2026-01-01_00-00-00.hoot'), [byte[]]@(2)
    )
    $run = Invoke-CounterProcess @(
        "`"$folder`"", '-OwletPath', "`"$fake`""
    )
    Assert-Equal 0 $run.ExitCode
    Assert-Equal 1 ([regex]::Matches(
        $run.Output, 'Brownouts: 1 \(estimated from Hoot\)'
    ).Count)
}

Invoke-Test 'CLI detailed mode prints event and fault information' {
    $file = Join-Path $testRoot 'cli-detail.wpilog'
    New-TestWpiLog $file @(
        [pscustomobject]@{
            Name = '/SystemStats/BrownedOut'
            Type = 'boolean'
            Samples = @(
                [pscustomobject]@{ Timestamp = 0; Value = $false }
                [pscustomobject]@{ Timestamp = 1000000; Value = $true }
            )
        }
    )
    $run = Invoke-CounterProcess @("`"$file`"", '-Detailed')
    Assert-Equal 0 $run.ExitCode
    Assert-True ($run.Output -match '1\.000000 s')
}

Invoke-Test 'CLI exits nonzero for an unsupported WPILOG' {
    $file = Join-Path $testRoot 'unsupported.wpilog'
    New-TestWpiLog $file @(
        [pscustomobject]@{
            Name = '/Unrelated'
            Type = 'boolean'
            Samples = @([pscustomobject]@{ Timestamp = 0; Value = $false })
        }
    )
    $run = Invoke-CounterProcess @("`"$file`"")
    Assert-Equal 1 $run.ExitCode
    Assert-True ($run.Error -match 'supported brownout')
}
```

- [ ] **Step 2: Run tests and verify CLI tests fail**

Run the full test command.

Expected: 17 tests pass and the CLI tests fail because the script has no main orchestration/output yet.

- [ ] **Step 3: Implement orchestration and bounded temporary cleanup**

Append:

```powershell
function New-ConversionDirectory {
    $root = [IO.Path]::GetTempPath()
    $path = Join-Path $root ('FRCBrownoutCounter-' + [guid]::NewGuid().ToString('N'))
    [void](New-Item -ItemType Directory -Path $path)
    return $path
}

function Remove-ConversionDirectory {
    param([string]$LiteralPath)
    $full = [IO.Path]::GetFullPath($LiteralPath)
    $temp = [IO.Path]::GetFullPath([IO.Path]::GetTempPath())
    if (-not $full.StartsWith($temp, [StringComparison]::OrdinalIgnoreCase) -or
        -not [IO.Path]::GetFileName($full).StartsWith('FRCBrownoutCounter-')) {
        throw "Refusing to remove unexpected conversion directory: $full"
    }
    if (Test-Path -LiteralPath $full) {
        Get-ChildItem -LiteralPath $full -File |
            ForEach-Object { Remove-Item -LiteralPath $_.FullName -Force }
        Remove-Item -LiteralPath $full -Force
    }
}

function Invoke-RobotBrownoutCounter {
    param(
        [string[]]$RequestedPaths,
        [string]$RequestedOwletPath
    )
    $inventory = Get-InputInventory $RequestedPaths
    $results = New-Object 'Collections.Generic.List[object]'
    $errors = New-Object 'Collections.Generic.List[string]'

    foreach ($wpiLog in $inventory.WpiLogs) {
        try {
            $signals = Read-WpiLogSignals $wpiLog
            if ($signals.HasBrownedOutEntry) {
                $results.Add((Get-ExactBrownoutAnalysis $signals $wpiLog))
            } elseif ($signals.HasRobotModeEntry -or $signals.HasRobotEnableEntry) {
                $results.Add((Get-HootBrownoutAnalysis @($signals) $wpiLog))
            } else {
                throw "No supported brownout or Hoot global signals found in $wpiLog"
            }
        } catch {
            $errors.Add($_.Exception.Message)
        }
    }

    if ($inventory.HootSessions.Count -gt 0) {
        try {
            $owlet = Resolve-Owlet -RequestedPath $RequestedOwletPath
        } catch {
            foreach ($session in $inventory.HootSessions) {
                $errors.Add("$($session.Source): $($_.Exception.Message)")
            }
            $owlet = $null
        }
        if ($null -ne $owlet) {
            foreach ($session in $inventory.HootSessions) {
                $temporary = New-ConversionDirectory
                try {
                    $sets = @()
                    $conversionWarnings = @()
                    [int]$index = 0
                    foreach ($hoot in $session.Paths) {
                        $index++
                        $output = Join-Path $temporary "bus-$index.wpilog"
                        $conversion = Invoke-OwletConversion $owlet $hoot $output
                        if ($conversion.Warning) {
                            $conversionWarnings += $conversion.Warning
                        }
                        $sets += Read-WpiLogSignals $output
                    }
                    $result = Get-HootBrownoutAnalysis $sets $session.Source
                    $result.Warnings = @(
                        $result.Warnings + $conversionWarnings | Select-Object -Unique
                    )
                    $results.Add($result)
                } catch {
                    $errors.Add("$($session.Source): $($_.Exception.Message)")
                } finally {
                    Remove-ConversionDirectory $temporary
                }
            }
        }
    }

    return [pscustomobject]@{
        Results = @($results)
        Errors = @($errors)
    }
}

function Format-BrownoutResult {
    param($Result, [switch]$IncludeDetails, [switch]$IncludeSource)
    $lines = New-Object 'Collections.Generic.List[string]'
    if ($IncludeSource) { $lines.Add("Log: $($Result.Source)") }
    if ($Result.Confidence -eq 'exact') {
        $lines.Add("Brownouts: $($Result.Count) (exact)")
    } else {
        $lines.Add("Brownouts: $($Result.Count) (estimated from Hoot)")
    }
    $lines.Add("Method: $($Result.Method)")
    if ($IncludeDetails) {
        foreach ($event in $Result.Events) {
            $lines.Add(
                ('Event: {0:F6} s' -f ([double]$event.Timestamp / 1000000.0))
            )
        }
        if ($Result.FaultControllers.Count -gt 0) {
            $lines.Add(
                'Controllers with bridge-brownout evidence: ' +
                ($Result.FaultControllers -join ', ')
            )
        }
        foreach ($warning in $Result.Warnings) {
            $lines.Add("Warning: $warning")
        }
    }
    return @($lines)
}

if ($MyInvocation.InvocationName -ne '.') {
    try {
        if ($null -eq $Path -or $Path.Count -eq 0) {
            throw 'Provide at least one .wpilog, .hoot, or directory path.'
        }
        $run = Invoke-RobotBrownoutCounter $Path $OwletPath
        $includeSource = ($run.Results.Count + $run.Errors.Count) -gt 1
        foreach ($result in $run.Results) {
            Format-BrownoutResult $result -IncludeDetails:$Detailed `
                -IncludeSource:$includeSource
            if ($includeSource) { '' }
        }
        foreach ($message in $run.Errors) {
            [Console]::Error.WriteLine("Error: $message")
        }
        if ($run.Errors.Count -gt 0) { exit 1 }
        exit 0
    } catch {
        [Console]::Error.WriteLine("Error: $($_.Exception.Message)")
        exit 1
    }
}
```

- [ ] **Step 4: Run all tests and verify the complete test suite is green**

Run the full test command.

Expected: `21 passed, 0 failed`.

- [ ] **Step 5: Run PowerShell syntax validation**

Run:

```powershell
$errors = $null
[void][Management.Automation.Language.Parser]::ParseFile(
  (Resolve-Path tools\Count-RobotBrownouts.ps1),
  [ref]$null,
  [ref]$errors
)
if ($errors.Count -ne 0) { $errors; exit 1 }
```

Expected: no output, exit code 0.

- [ ] **Step 6: Commit the CLI**

```powershell
git add -- tools\Count-RobotBrownouts.ps1 tools\tests\Count-RobotBrownouts.Tests.ps1
git commit -m "feat: add standalone robot brownout counter"
```

## Task 6: Verify the supplied exact WPILOG and raw Hoot files

**Files:**

- Test: `TRI Logs/WPILog Files/akit_26-06-27_21-02-44_txhou1_e13.wpilog`
- Test: the two raw `.hoot` files in `TRI Logs/Hoot Logs/TXHOU1_E13`
- Modify only if a failing integration test exposes a defect:
  `tools/tests/Count-RobotBrownouts.Tests.ps1`
  and `tools/Count-RobotBrownouts.ps1`

- [ ] **Step 1: Run the exact E13 WPILOG acceptance check**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File tools\Count-RobotBrownouts.ps1 `
  'TRI Logs\WPILog Files\akit_26-06-27_21-02-44_txhou1_e13.wpilog' `
  -Detailed
```

Expected:

```text
Brownouts: 44 (exact)
Method: ExactRoboRioBrownedOut
```

Detailed output must also contain the partial-final-record warning.

- [ ] **Step 2: Prove raw Hoot support without sibling conversions**

Copy only the two raw E13 Hoots into a uniquely named temporary directory, invoke the script on that directory, and clean up:

```powershell
$source = Resolve-Path 'TRI Logs\Hoot Logs\TXHOU1_E13'
$temporary = Join-Path ([IO.Path]::GetTempPath()) (
  'FRCBrownoutAcceptance-' + [guid]::NewGuid().ToString('N')
)
[void](New-Item -ItemType Directory -Path $temporary)
try {
  Get-ChildItem -LiteralPath $source -Filter *.hoot |
    ForEach-Object { Copy-Item -LiteralPath $_.FullName -Destination $temporary }
  powershell.exe -NoProfile -ExecutionPolicy Bypass `
    -File tools\Count-RobotBrownouts.ps1 $temporary -Detailed
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
} finally {
  Get-ChildItem -LiteralPath $temporary -File |
    ForEach-Object { Remove-Item -LiteralPath $_.FullName -Force }
  Remove-Item -LiteralPath $temporary -Force
}
```

Expected:

```text
Brownouts: 34 (estimated from Hoot)
Method: EstimatedHootEnableInterruptions
```

There must be one logical result for the paired files and controller bridge-brownout evidence in detailed output.

- [ ] **Step 3: If either acceptance check fails, add a focused regression test first**

The regression test must reproduce the observed parser, grouping, conversion, or semantic failure using the smallest synthetic fixture possible. Run it and confirm the expected failure before changing production code. Then make the smallest correction and rerun the complete test suite.

- [ ] **Step 4: Run fresh full verification**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File tools\tests\Count-RobotBrownouts.Tests.ps1

git diff --check

.\gradlew.bat test
```

Expected:

- `21 passed, 0 failed`;
- `git diff --check` exits 0 with no output; and
- Gradle reports `BUILD SUCCESSFUL`.

- [ ] **Step 5: Inspect the final scope**

Run:

```powershell
git status --short
git diff --stat HEAD~1
git diff -- tools\Count-RobotBrownouts.ps1 `
  tools\tests\Count-RobotBrownouts.Tests.ps1
```

Confirm only the counter, its tests, and the approved planning documents are part of this work. Do not stage `.jqwik-database` or `TRI Logs`.

- [ ] **Step 6: Commit any integration-driven correction**

Only if Step 3 required a correction:

```powershell
git add -- tools\Count-RobotBrownouts.ps1 tools\tests\Count-RobotBrownouts.Tests.ps1
git commit -m "fix: handle real robot brownout logs"
```
