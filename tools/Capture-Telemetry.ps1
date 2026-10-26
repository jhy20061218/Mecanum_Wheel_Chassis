# JHY - STM32 USART1 telemetry capture. Does not send motion commands.
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Port,
    [ValidateRange(9600, 2000000)][int]$Baud = 420000,
    [ValidateRange(1, 3600)][int]$DurationSec = 30,
    [string]$Output = "hardware_telemetry.csv"
)
$ErrorActionPreference = 'Stop'
$destination = [System.IO.Path]::GetFullPath($Output)
$parentFolder = [System.IO.Path]::GetDirectoryName($destination)
if (-not (Test-Path -LiteralPath $parentFolder)) {
    New-Item -ItemType Directory -Path $parentFolder -Force | Out-Null
}
if (Test-Path -LiteralPath $destination) { throw "Output exists: $destination. Choose a new filename." }
$serialPort = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$serialPort.ReadTimeout = 300
$serialPort.NewLine = "`n"
$writer = $null
$rawWriter = $null
$recordCount = 0
$badCount = 0
$headerFields = 0
$clock = [System.Diagnostics.Stopwatch]::new()
try {
    $serialPort.Open()
    $writer = [System.IO.StreamWriter]::new($destination, $false, [System.Text.UTF8Encoding]::new($false))
    $rawWriter = [System.IO.StreamWriter]::new($destination + '.raw.txt', $false, [System.Text.UTF8Encoding]::new($false))
    $clock.Start()
    Write-Host "Listening on $Port at $Baud baud for $DurationSec seconds; no commands are transmitted."
    while ($clock.Elapsed.TotalSeconds -lt $DurationSec) {
        try { $line = $serialPort.ReadLine().Trim() } catch [System.TimeoutException] { continue }
        $rawWriter.WriteLine($line)
        if ([string]::IsNullOrWhiteSpace($line)) { continue }
        $parts = $line.Split(',')
        if ($headerFields -eq 0) {
            # Firmware prints a header on startup and periodically. Numeric data
            # received before the first complete header remain in the raw log.
            if ($line -match '^(ms|time_ms|tick_ms|t_ms),') {
                $headerFields = $parts.Length
                $writer.WriteLine($line)
            }
            continue
        }
        if ($line -match '^(ms|time_ms|tick_ms|t_ms),') { continue }
        if ($parts.Length -ne $headerFields -or $parts[0] -notmatch '^\d+$') { $badCount++; continue }
        $writer.WriteLine($line)
        $recordCount++
        if (($recordCount % 50) -eq 0) { $writer.Flush(); Write-Host "Captured $recordCount records" }
    }
} finally {
    if ($null -ne $writer) { $writer.Dispose() }
    if ($null -ne $rawWriter) { $rawWriter.Dispose() }
    if ($serialPort.IsOpen) { $serialPort.Close() }
    $serialPort.Dispose()
}
$metadata = [ordered]@{
    author = 'JHY'; source = 'STM32 USART1 hardware telemetry'; port = $Port
    baud = $Baud; captured_at_local = (Get-Date -Format 'yyyy-MM-ddTHH:mm:ssK')
    requested_seconds = $DurationSec; actual_seconds = $clock.Elapsed.TotalSeconds
    valid_rows = $recordCount; malformed_rows = $badCount; header_fields = $headerFields
    file = $destination; raw_file = $destination + '.raw.txt'
}
$metadata | ConvertTo-Json | Set-Content -LiteralPath ($destination + '.metadata.json') -Encoding UTF8
if ($headerFields -eq 0) { throw 'No complete CSV header received. Capture for at least 6 seconds to receive the periodic 5-second header; inspect baud rate and wiring.' }
if ($recordCount -eq 0) { throw 'No telemetry records received. Inspect the raw log and wiring.' }
Write-Host "Saved $recordCount records to $destination; malformed lines: $badCount"
