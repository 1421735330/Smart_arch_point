$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (-not (Test-Path ".venv\Scripts\python.exe")) {
    python -m venv .venv
}

& ".venv\Scripts\python.exe" -m pip install -r requirements-build.txt
& ".venv\Scripts\python.exe" -m PyInstaller `
    --noconfirm `
    --clean `
    --onefile `
    --windowed `
    --name SmartPlacePoint `
    --distpath dist `
    --workpath build `
    --specpath build `
    app.pyw

Write-Host "Build complete: $PSScriptRoot\dist\SmartPlacePoint.exe"
