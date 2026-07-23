<#
    Client-side launcher: run the city-preparation pipeline on the remote server.

    Reads the SSH host / key / remote base directory from server.properties (the same file the
    Java remote-run uses), prompts for the city and stage options exactly like build_city.bat,
    then SSHes into the server, pulls the repo, and runs pipeline/00_city_preparation.py there
    inside the pedsimcity-prep conda environment. Output streams back live.

    Invoked by build_city_remote.bat (double-click), or directly:  powershell -File remote_prep.ps1
#>

$ErrorActionPreference = 'Stop'

$RepoRoot  = Split-Path $PSScriptRoot -Parent
$PropsFile = Join-Path $RepoRoot 'server.properties'

if (-not (Test-Path $PropsFile)) {
    Write-Host "Missing server.properties at repo root:" -ForegroundColor Red
    Write-Host "  $PropsFile"
    Write-Host "Copy server.properties.example to server.properties and fill in your host/key."
    exit 1
}

# --- parse the Java .properties file ---------------------------------------
$props = @{}
foreach ($line in Get-Content -LiteralPath $PropsFile) {
    $trimmed = $line.Trim()
    if ($trimmed -eq '' -or $trimmed.StartsWith('#') -or $trimmed.StartsWith('!')) { continue }
    $idx = $trimmed.IndexOf('=')
    if ($idx -lt 1) { continue }
    $key = $trimmed.Substring(0, $idx).Trim()
    $val = $trimmed.Substring($idx + 1).Trim()
    # Java escapes backslashes by doubling them; collapse back to single for real paths.
    $val = $val -replace '\\\\', '\'
    $props[$key] = $val
}

function Prop([string]$name, [string]$default) {
    if ($props.ContainsKey($name) -and $props[$name] -ne '') { return $props[$name] }
    return $default
}

$sshExe     = Prop 'ssh.executable' 'ssh'
$sshKey     = Prop 'ssh.key' ''
$serverHost = Prop 'server.host' ''
$baseDir    = Prop 'server.projectBaseDir' ''
$repoUrl    = Prop 'server.repoUrl' 'https://github.com/g-filomena/PedSimCity.git'

if ($serverHost -eq '') {
    Write-Host "server.host is not set in server.properties." -ForegroundColor Red
    exit 1
}

# The pipeline lives in the core PedSimCity checkout (mirrors remoteProjectDir('PedSimCity')).
if ($baseDir -ne '') { $remoteDir = "$baseDir/PedSimCity" } else { $remoteDir = 'PedSimCity' }

# --- collect run parameters (mirrors build_city.bat) -----------------------
$city = Read-Host 'Enter city name (e.g. Torino)'
if ($city -eq '') { Write-Host 'No city name entered. Exiting.'; exit 1 }

Write-Host ''
Write-Host 'OSM place query and EPSG are saved server-side in prep_config.json after the first run.'
Write-Host 'Leave both blank to reuse the saved values; fill them in for a first-time city.'
$place = Read-Host 'OSM place query (e.g. Torino, Italy) [blank = reuse saved]'
$epsg  = Read-Host 'Projected EPSG code (e.g. 3003) [blank = reuse saved]'

Write-Host ''
Write-Host 'Which stages?'
Write-Host '  [1] everything (default)'
Write-Host '  [2] base layers: buildings,network,districts,elevation,barriers,pois'
Write-Host '  [3] landmarks:   buildings,network,elevation,sightlines,landmarks'
Write-Host '  [4] custom stage list'
$stageChoice = Read-Host 'Choice [1]'
$stages = ''
switch ($stageChoice) {
    '2' { $stages = 'buildings,network,districts,elevation,barriers,pois' }
    '3' { $stages = 'buildings,network,elevation,sightlines,landmarks' }
    '4' {
        $stages = Read-Host 'Enter stages (comma-separated: buildings,network,districts,elevation,barriers,pois,sightlines,landmarks)'
        if ($stages -eq '') { Write-Host 'No stages entered. Exiting.'; exit 1 }
    }
}

Write-Host ''
$consolidate = Read-Host 'Consolidate network nodes? yes/no [yes]'
if ($consolidate -eq '') { $consolidate = 'yes' }
$consTol = ''
if ($consolidate -notmatch '^(no|n)$') {
    $consTol = Read-Host 'Consolidation tolerance in metres [15]'
    if ($consTol -eq '') { $consTol = '15' }
}

# --- build the python argument list ----------------------------------------
$pyArgs = @('--city', $city)
if ($place -ne '') { $pyArgs += @('--place', $place) }
if ($epsg  -ne '') { $pyArgs += @('--epsg', $epsg) }
if ($stages -ne '') { $pyArgs += @('--stages', $stages) }
$pyArgs += @('--consolidate-network', $consolidate)
if ($consTol -ne '') { $pyArgs += @('--consolidate-tolerance', $consTol) }

# Single-quote each argument for the remote bash shell (handles spaces in the place name).
function BashQuote([string]$s) { "'" + ($s -replace "'", "'\''") + "'" }
$remoteArgs = ($pyArgs | ForEach-Object { BashQuote $_ }) -join ' '

$remoteDirQ = BashQuote $remoteDir
$repoUrlQ   = BashQuote $repoUrl
$cityQ      = BashQuote $city

# One SSH invocation for a given remote command; returns its exit code.
$sshBase = @()
if ($sshKey -ne '') { $sshBase += @('-i', $sshKey) }
function Invoke-Ssh([string]$cmd) {
    $a = @() + $sshBase + $serverHost + $cmd
    & $sshExe @a
    return $LASTEXITCODE
}

Write-Host ''
Write-Host "[SERVER] $serverHost" -ForegroundColor Cyan

# --- phase 1: clone on first use, pull latest, ensure inputData/<City> exists ---
$prepCmd = "if [ ! -d $remoteDirQ/.git ]; then echo '>> cloning repo' && git clone $repoUrlQ $remoteDirQ; fi && " +
           "cd $remoteDirQ && echo '>> pulling repo' && git pull --ff-only && mkdir -p inputData/$cityQ"
Write-Host '>> Preparing server checkout ...'
$code = Invoke-Ssh $prepCmd
if ($code -ne 0) {
    Write-Host "Server checkout prep (clone/pull) failed with code $code." -ForegroundColor Red
    exit $code
}

# --- phase 2: upload the whole local inputData\<City> (rasters, official layers, config) ---
# scp treats a local "C:\..." as host:path, so we run it from the repo root with a colon-free
# relative path. This carries the files git does not (gitignored rasters, LFS-only .gpkg, etc.).
$localCity = Join-Path $RepoRoot "inputData\$city"
if (Test-Path -LiteralPath $localCity) {
    $bytes = (Get-ChildItem -LiteralPath $localCity -Recurse -File -Force |
              Measure-Object -Property Length -Sum).Sum
    $mb = [math]::Round(($bytes / 1MB), 1)
    Write-Host ''
    $up = Read-Host "Upload local inputData\$city ($mb MB) to the server? yes/no [yes]"
    if ($up -eq '') { $up = 'yes' }
    if ($up -notmatch '^(no|n)$') {
        if ($sshExe -eq 'ssh') {
            $scpExe = 'scp'
        } else {
            $scpExe = Join-Path (Split-Path $sshExe -Parent) 'scp.exe'
        }
        $scpArgs = @('-r')
        if ($sshKey -ne '') { $scpArgs += @('-i', $sshKey) }
        $scpArgs += "inputData/$city"
        $scpArgs += "${serverHost}:$remoteDir/inputData/"
        Write-Host ">> Uploading inputData/$city ..." -ForegroundColor Cyan
        Push-Location $RepoRoot
        try { & $scpExe @scpArgs } finally { Pop-Location }
        if ($LASTEXITCODE -ne 0) {
            Write-Host "scp upload failed with code $LASTEXITCODE." -ForegroundColor Red
            exit $LASTEXITCODE
        }
    } else {
        Write-Host '>> Skipping input upload.'
    }
} else {
    Write-Host ''
    Write-Host "No local inputData\$city folder - nothing to upload (OSM-only city)."
}

# --- phase 3: run the pipeline ---------------------------------------------
$runCmd = "cd $remoteDirQ && bash pipeline/run_prep_remote.sh $remoteArgs"
Write-Host ''
Write-Host "[CMD] $runCmd" -ForegroundColor DarkGray
Write-Host ''
$code = Invoke-Ssh $runCmd
Write-Host ''
if ($code -ne 0) {
    Write-Host "Remote pipeline exited with code $code." -ForegroundColor Red
    exit $code
}
Write-Host "Prep finished. Outputs are in $remoteDir/src/main/resources/$city on the server." -ForegroundColor Green

# --- phase 4: download the produced resources back to the local checkout -----
# Only needed if you run the simulation (or inspect the layers) locally; skip it when the sim
# runs on the same server. Same colon-free trick: scp from the repo root into a relative dest.
Write-Host ''
$dl = Read-Host "Download the produced src\main\resources\$city to this machine? yes/no [yes]"
if ($dl -eq '') { $dl = 'yes' }
if ($dl -notmatch '^(no|n)$') {
    if ($sshExe -eq 'ssh') {
        $scpExe = 'scp'
    } else {
        $scpExe = Join-Path (Split-Path $sshExe -Parent) 'scp.exe'
    }
    New-Item -ItemType Directory -Force -Path (Join-Path $RepoRoot 'src\main\resources') | Out-Null
    $dlArgs = @('-r')
    if ($sshKey -ne '') { $dlArgs += @('-i', $sshKey) }
    $dlArgs += "${serverHost}:$remoteDir/src/main/resources/$city"
    $dlArgs += 'src/main/resources/'
    Write-Host ">> Downloading src/main/resources/$city ..." -ForegroundColor Cyan
    Push-Location $RepoRoot
    try { & $scpExe @dlArgs } finally { Pop-Location }
    if ($LASTEXITCODE -ne 0) {
        Write-Host "scp download failed with code $LASTEXITCODE." -ForegroundColor Red
        exit $LASTEXITCODE
    }
    Write-Host "Downloaded to src\main\resources\$city." -ForegroundColor Green
} else {
    Write-Host '>> Skipping download. Outputs remain on the server.'
}
exit 0
