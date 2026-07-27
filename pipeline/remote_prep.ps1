<#
    Client-side launcher: run the city-preparation pipeline on the remote server.

    Reads the SSH host / key / remote base directory from server.properties (the same file the
    Java remote-run uses), prompts for the city and stage options like build_city.bat, then SSHes
    into the server, pulls the repo, uploads the city's inputs, and runs the pipeline there in the
    pedsimcity-prep conda environment.

    The pipeline runs DETACHED on the server (setsid), so it survives the laptop hibernating or the
    SSH connection dropping. The launcher follows the log live; if you get disconnected, the run
    keeps going on the server and re-running the launcher for the same city RECONNECTS to it.

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

# --- SSH helpers ------------------------------------------------------------
# Single-quote a value for the remote bash shell (handles spaces in paths / the place name).
function BashQuote([string]$s) { "'" + ($s -replace "'", "'\''") + "'" }

$sshBase = @()
if ($sshKey -ne '') { $sshBase += @('-i', $sshKey) }

# Stream the server's output straight to the console (do NOT assign the result); read $LASTEXITCODE.
function Invoke-Ssh([string]$cmd) {
    $a = @() + $sshBase + $serverHost + $cmd
    & $sshExe @a
}
# Capture the server's stdout lines (for small queries: status checks, file sizes).
function Invoke-SshCapture([string]$cmd) {
    $a = @() + $sshBase + $serverHost + $cmd
    return (& $sshExe @a 2>$null)
}

function Get-ScpExe {
    if ($sshExe -eq 'ssh') { return 'scp' }
    return (Join-Path (Split-Path $sshExe -Parent) 'scp.exe')
}

# Download the produced resources back to the local checkout (prompted). Used after a run finishes.
function Invoke-ResourceDownload {
    Write-Host ''
    $dl = Read-Host "Download the produced src\main\resources\$city to this machine? yes/no [yes]"
    if ($dl -eq '') { $dl = 'yes' }
    if ($dl -match '^(no|n)$') {
        Write-Host '>> Skipping download. Outputs remain on the server.'
        return
    }
    $scpExe = Get-ScpExe
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
}

# After the attach/tail returns, read the recorded exit code and either download or report.
function Complete-Run {
    $exitOut  = Invoke-SshCapture "if [ -f $exitAbs ]; then echo EXIT:`$(cat $exitAbs); else echo STILLRUNNING; fi"
    $exitLine = ($exitOut | Where-Object { $_ -match '^(EXIT:|STILLRUNNING)' } | Select-Object -First 1)
    Write-Host ''
    if ($exitLine -match '^EXIT:(-?\d+)') {
        $pipeCode = [int]$matches[1]
        if ($pipeCode -eq 0) {
            Write-Host "Prep finished. Outputs are in $remoteDir/src/main/resources/$city on the server." -ForegroundColor Green
            Invoke-ResourceDownload
            exit 0
        }
        Write-Host "Remote pipeline failed (exit $pipeCode)." -ForegroundColor Red
        Write-Host "Full log on the server: $remoteDir/inputData/$city/prep_remote.log"
        exit $pipeCode
    }
    # No exit file yet -> the follow ended without the job finishing (we were disconnected).
    Write-Host "Disconnected - but the run is still going on the server." -ForegroundColor Yellow
    Write-Host "Re-run build_city_remote.bat for $city to reconnect and keep following it."
    exit 0
}

# --- city + remote paths ----------------------------------------------------
$city = Read-Host 'Enter city name (e.g. Torino)'
if ($city -eq '') { Write-Host 'No city name entered. Exiting.'; exit 1 }

$cityQ      = BashQuote $city
$remoteDirQ = BashQuote $remoteDir
$repoUrlQ   = BashQuote $repoUrl
# Detached-run bookkeeping files (absolute, bash-quoted) kept per-city under inputData/<City>/.
$logAbs    = BashQuote "$remoteDir/inputData/$city/prep_remote.log"
$pidAbs    = BashQuote "$remoteDir/inputData/$city/prep_remote.pid"
$exitAbs   = BashQuote "$remoteDir/inputData/$city/prep_remote.exit"
$scriptAbs = BashQuote "$remoteDir/pipeline/run_prep_remote.sh"

# Follow the log until the detached job's PID (bash var $P, set by each caller) dies. `tail --pid`
# is unreliable with inotify - it can block after the log stops changing and never notice the PID
# is gone - so run `tail -f` in the background and poll `kill -0` instead. No double quotes: Windows
# PowerShell strips embedded " when passing an argument to ssh.exe.
$follow = "tail -f $logAbs & TP=`$!; while kill -0 `$P 2>/dev/null; do sleep 2; done; sleep 1; kill `$TP 2>/dev/null"

Write-Host ''
Write-Host "[SERVER] $serverHost" -ForegroundColor Cyan

# --- reconnect check: is a detached run already going for this city? --------
# NB: no double quotes in remote command strings - Windows PowerShell strips embedded " when
# handing an argument to ssh.exe. Single quotes survive; bash vars are numeric/paths so unquoted
# is safe. ${P:-x} defaults to a non-numeric sentinel so `kill -0` cleanly fails when idle.
$checkOut = Invoke-SshCapture "P=`$(cat $pidAbs 2>/dev/null); P=`${P:-x}; if kill -0 `$P 2>/dev/null; then echo RUNNING:`$P; else echo IDLE; fi"
$isRunning = [bool]($checkOut | Where-Object { $_ -match '^RUNNING:' })

if ($isRunning) {
    Write-Host "A prep run for $city is already running on the server - reconnecting." -ForegroundColor Cyan
    Write-Host '(Following the log; safe to disconnect again - the run continues.)'
    Write-Host ''
    Invoke-Ssh "P=`$(cat $pidAbs); echo '>> Reconnecting to running job. Following log:'; $follow"
    Complete-Run
}

# =====================  fresh run  =====================
# --- collect run parameters (mirrors build_city.bat) -----------------------
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
$remoteArgs = ($pyArgs | ForEach-Object { BashQuote $_ }) -join ' '

# --- phase 1: clone on first use, pull latest, ensure inputData/<City> exists & is writable ---
# chmod -R u+rwX heals a directory left non-writable by an earlier `scp -r` from Windows (scp can
# translate the Windows ACL into a mode with no owner-write bit), which would block the detached
# run from creating its log/pid files.
$prepCmd = "if [ ! -d $remoteDirQ/.git ]; then echo '>> cloning repo' && git clone $repoUrlQ $remoteDirQ; fi && " +
           "cd $remoteDirQ && echo '>> pulling repo' && git pull --ff-only && " +
           "mkdir -p inputData/$cityQ && chmod -R u+rwX inputData/$cityQ"
Write-Host ''
Write-Host '>> Preparing server checkout ...'
Invoke-Ssh $prepCmd
if ($LASTEXITCODE -ne 0) {
    Write-Host "Server checkout prep (clone/pull) failed with code $LASTEXITCODE." -ForegroundColor Red
    exit $LASTEXITCODE
}

# --- phase 2: upload only the input files the pipeline actually reads -------
# The inputData\<City> folder also holds QGIS projects, PDFs, raw source layers and prep_staging/
# checkpoints the pipeline never reads, so we curate to the documented input set (rasters,
# official / height / study layers, the reused obstructions cache, prep_config.json) and skip any
# file already on the server with a matching size. scp treats a local "C:\..." as host:path, so
# uploads run from the repo root with colon-free relative paths.
$localCity = Join-Path $RepoRoot "inputData\$city"
if (-not (Test-Path -LiteralPath $localCity)) {
    Write-Host ''
    Write-Host "No local inputData\$city folder - nothing to upload (OSM-only city)."
} else {
    # The exact set _find_raw / _find_raster / the obstructions cache look for in 00_city_preparation.py.
    $patterns = @(
        "${city}_DTM.*", "${city}_DEM.*", "${city}_DSM.*",
        "${city}_officialBuildings.gpkg", "${city}_detailedBuildings.gpkg",
        "${city}_buildingHeights.gpkg", "${city}_studyArea.gpkg",
        "${city}_obstructions.gpkg", "prep_config.json"
    )
    $candidates = @()
    foreach ($pat in $patterns) {
        $candidates += Get-ChildItem -LiteralPath $localCity -Filter $pat -File -Force -ErrorAction SilentlyContinue
    }
    $candidates = @($candidates | Sort-Object -Property FullName -Unique)

    if ($candidates.Count -eq 0) {
        Write-Host ''
        Write-Host "No pipeline input files found in inputData\$city - nothing to upload (OSM-only run)."
    } else {
        # Ask the server which inputs it already has (name + size) so unchanged ones are skipped.
        # find -printf avoids embedded double quotes (which PowerShell would strip); %f is the basename.
        $statCmd = "cd $remoteDirQ/inputData/$cityQ 2>/dev/null && find . -maxdepth 1 -type f -printf '%s %f\n'"
        $statOut = Invoke-SshCapture $statCmd
        $remoteSizes = @{}
        foreach ($line in $statOut) {
            if ($line -match '^\s*(\d+)\s+(.+?)\s*$') { $remoteSizes[$matches[2]] = [int64]$matches[1] }
        }

        $toUpload = @()
        $skipped = 0
        foreach ($f in $candidates) {
            $onServer = $remoteSizes.ContainsKey($f.Name)
            # prep_config.json is server-managed: the pipeline rewrites it (place/EPSG/method) on every
            # run, so its bytes drift from the local copy after any server run. Don't nag or clobber it -
            # upload only to SEED a server that has none; CLI --place/--epsg win over it anyway.
            if ($f.Name -eq 'prep_config.json') {
                if ($onServer) { $skipped++ } else { $toUpload += $f }
                continue
            }
            if ($onServer -and $remoteSizes[$f.Name] -eq $f.Length) {
                $skipped++
            } else {
                $toUpload += $f
            }
        }

        Write-Host ''
        if ($skipped -gt 0) {
            Write-Host "$skipped input file(s) already on the server with matching size - skipping."
        }

        if ($toUpload.Count -eq 0) {
            Write-Host 'All pipeline inputs already present on the server. Nothing to upload.'
        } else {
            $upMb = [math]::Round(((($toUpload | Measure-Object -Property Length -Sum).Sum) / 1MB), 1)
            Write-Host "To upload ($($toUpload.Count) file(s), $upMb MB):"
            foreach ($f in $toUpload) {
                Write-Host ("  {0}  ({1} MB)" -f $f.Name, [math]::Round($f.Length / 1MB, 1))
            }
            $up = Read-Host 'Upload these to the server? yes/no [yes]'
            if ($up -eq '') { $up = 'yes' }
            if ($up -notmatch '^(no|n)$') {
                $scpExe = Get-ScpExe
                $scpArgs = @()
                if ($sshKey -ne '') { $scpArgs += @('-i', $sshKey) }
                foreach ($f in $toUpload) { $scpArgs += "inputData/$city/$($f.Name)" }
                $scpArgs += "${serverHost}:$remoteDir/inputData/$city/"
                Write-Host ">> Uploading $($toUpload.Count) file(s) ..." -ForegroundColor Cyan
                Push-Location $RepoRoot
                try { & $scpExe @scpArgs } finally { Pop-Location }
                if ($LASTEXITCODE -ne 0) {
                    Write-Host "scp upload failed with code $LASTEXITCODE." -ForegroundColor Red
                    exit $LASTEXITCODE
                }
            } else {
                Write-Host '>> Skipping input upload.'
            }
        }
    }
}

# --- phase 3: start the pipeline DETACHED, then follow its log --------------
# setsid puts the run in its own session (no controlling terminal), redirecting all output to a log
# file. That decouples it from this SSH connection, so hibernating the laptop or dropping the link
# does NOT kill it. We record its PID and follow the log with `tail --pid`, which exits exactly when
# the run finishes. run_prep_remote.sh writes its exit code to $exitAbs (via PREP_EXIT_FILE).
$startCmd = ": > $logAbs; rm -f $exitAbs; " +
            "setsid env PREP_EXIT_FILE=$exitAbs bash $scriptAbs $remoteArgs > $logAbs 2>&1 < /dev/null & " +
            "P=`$!; echo `$P > $pidAbs; " +
            "echo '>> Started detached. PID:' `$P; " +
            "echo '>> Safe to hibernate/disconnect - the run continues on the server.'; " +
            "echo '>> If disconnected, re-run build_city_remote.bat for this city to reconnect. Following log:'; " +
            $follow
Write-Host ''
Write-Host '>> Launching detached run ...' -ForegroundColor Cyan
Invoke-Ssh $startCmd
Complete-Run
