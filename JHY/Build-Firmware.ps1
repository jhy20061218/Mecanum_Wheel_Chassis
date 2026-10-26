param(
    [string]$CMake = 'cmake',
    [string]$Ninja = 'ninja',
    [ValidateSet('Debug', 'Release')][string]$Configuration = 'Debug'
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$cmakePath = (Get-Command $CMake -ErrorAction Stop).Source
$ninjaPath = (Get-Command $Ninja -ErrorAction Stop).Source
$null = Get-Command arm-none-eabi-gcc -ErrorAction Stop
$drive = @('T','U','V','W','X','Y','Z') | Where-Object {
    -not (Test-Path -LiteralPath "${_}:\")
} | Select-Object -First 1
if (-not $drive) { throw 'No unused T: through Z: drive is available for the temporary ASCII build path.' }
$mappedDrive = "${drive}:"
$buildRelative = "build/JHY_${Configuration}_${drive}"
$pushed = $false
$created = $false
try {
    & subst $mappedDrive $projectRoot
    if ($LASTEXITCODE -ne 0) { throw 'Cannot create temporary ASCII drive mapping.' }
    $created = $true
    Push-Location "${mappedDrive}/"
    $pushed = $true
    & $cmakePath -S "${mappedDrive}/" -B "${mappedDrive}/$buildRelative" -G Ninja "-DCMAKE_MAKE_PROGRAM=$ninjaPath" "-DCMAKE_TOOLCHAIN_FILE=${mappedDrive}/cmake/gcc-arm-none-eabi.cmake" "-DCMAKE_BUILD_TYPE=$Configuration"
    if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }
    & $cmakePath --build "${mappedDrive}/$buildRelative"
    if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed.' }
    Write-Output (Join-Path $projectRoot "$buildRelative/Mecanum_Wheel_Chassis.elf")
} finally {
    if ($pushed) { Pop-Location }
    # This mapping was created above only after verifying that the drive was unused.
    if ($created) { & subst $mappedDrive /D }
}
