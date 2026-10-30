param(
    [string]$Python = (Join-Path (Split-Path -Parent $PSScriptRoot) '.venv\Scripts\python.exe')
)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path -LiteralPath $Python -PathType Leaf)) { throw "测试Python不存在：$Python" }
Push-Location $taskRoot
try {
    New-Item -ItemType Directory -Force (Join-Path $taskRoot 'output\tests') | Out-Null
    & $Python -B -m unittest discover -s tests -v
    if ($LASTEXITCODE -ne 0) { throw '主机协议或Web回归未通过' }
    & $Python -B (Join-Path $PSScriptRoot 'check_project.py')
    if ($LASTEXITCODE -ne 0) { throw '固件当前阶段检查未通过' }
} finally {
    Pop-Location
}
