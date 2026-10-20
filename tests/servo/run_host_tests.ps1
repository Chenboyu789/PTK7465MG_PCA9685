# Run from a Visual Studio Developer PowerShell with the C/C++ tools available.
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $taskRoot
try {
    if (-not (Get-Command cl.exe -ErrorAction SilentlyContinue)) {
        throw 'Run from Visual Studio Developer PowerShell (C/C++ tools required).'
    }
    New-Item -ItemType Directory -Force -Path build/host-tests | Out-Null
    & cl.exe /nologo /std:c11 /W3 /WX /utf-8 `
        /Itests/servo/stubs /Itests/pca9685/stubs /Icomponents/pca9685/include `
        /Icomponents/servo/include /Icomponents/board/include `
        /Icomponents/application/include /Icomponents/application/private_include `
        components/servo/servo.c components/application/business_servo_cycle.c `
        components/application/app_scheduler.c components/application/app_profile.c components/application/app.c `
        tests/servo/test_application.c `
        /Fobuild/host-tests/ /Febuild/host-tests/test_application.exe
    if ($LASTEXITCODE -ne 0) { throw 'Servo host test compilation failed.' }
    & ./build/host-tests/test_application.exe
    if ($LASTEXITCODE -ne 0) { throw 'Servo host tests failed.' }
} finally {
    Pop-Location
}
