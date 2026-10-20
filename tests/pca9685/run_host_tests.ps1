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
        /Itests/pca9685/stubs /Icomponents/pca9685/include /Icomponents/board/include `
        components/pca9685/pca9685.c components/board/board.c tests/pca9685/test_pca9685.c `
        /Fobuild/host-tests/ /Febuild/host-tests/test_pca9685.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed.' }
    & ./build/host-tests/test_pca9685.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed.' }
} finally {
    Pop-Location
}
