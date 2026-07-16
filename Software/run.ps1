$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (-not (Test-Path ".venv\Scripts\pythonw.exe")) {
    throw "Environment not found. Run .\setup.ps1 first."
}

Start-Process -FilePath ".venv\Scripts\pythonw.exe" -ArgumentList "app.pyw" -WorkingDirectory $PSScriptRoot
