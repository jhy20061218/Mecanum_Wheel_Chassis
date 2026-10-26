param([switch]$RunAll)
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    New-Item -ItemType Directory -Force build,results | Out-Null
    & gcc -std=c11 -O2 -Wall -Wextra -Wpedantic -Werror -I../JHY/Inc -Isrc src/main.c src/config.c src/sim.c src/editor.c src/tests.c src/io.c ../JHY/Src/alg_mecanum_wheel_chassis.c -o build/mecanum_sim.exe -lm -lshell32
    if ($LASTEXITCODE -ne 0) { throw 'C compiler failed.' }
    & ./build/mecanum_sim.exe --self-test
    if ($LASTEXITCODE -ne 0) { throw 'Self-test failed.' }
    $cases = @('default')
    if ($RunAll) { $cases = @('default','slip','timeout','overheat','estop','saturation') }
    foreach ($case in $cases) {
        & ./build/mecanum_sim.exe --config "config/$case.cfg" --output "results/$case" --quiet
        if ($LASTEXITCODE -ne 0) { throw "Simulation failed: $case" }
    }
} finally { Pop-Location }
