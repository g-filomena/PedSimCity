@echo off
setlocal EnableExtensions

REM PedSimCity paired night-module comparison launcher.
REM Runs the night module twice on one city and diffs the trip diagnostics.
REM Prompts for the city, the population share, the number of days, and the two
REM arms - no city and no experiment is named in the Python script.
REM
REM The arms are command-line arguments passed straight to the simulation. The
REM default baseline, --maxKnownDarkEdgeCostMultiplier=1.0, is the documented
REM control for the planning-cost mechanism: 1.0 is the behaviour before darkness
REM reached route planning. Give the two arms different arguments - the script
REM refuses to compare a run with itself.
REM
REM Maven does the building, so no Conda environment is needed; it uses whatever
REM python is on PATH for the driver script.

rem This script lives in scripts/; ROOT is the repo root one level up.
set "ROOT=%~dp0..\"

where python >nul 2>nul
if errorlevel 1 (
    echo Python not found on PATH.
    pause
    exit /b 1
)

where mvn >nul 2>nul
if errorlevel 1 (
    echo Maven not found on PATH.
    pause
    exit /b 1
)

set /p CITY_NAME=Enter city name ^(e.g. Torino^):
if "%CITY_NAME%"=="" (
    echo No city name entered. Exiting.
    pause
    exit /b 1
)

if not exist "%ROOT%src\main\resources\%CITY_NAME%\%CITY_NAME%_nodes.gpkg" (
    echo No node layer for "%CITY_NAME%":
    echo "%ROOT%src\main\resources\%CITY_NAME%\%CITY_NAME%_nodes.gpkg"
    pause
    exit /b 1
)

set /p PERCENTAGE=Share of the census resident total [0.02]:
if "%PERCENTAGE%"=="" set "PERCENTAGE=0.02"

set /p DAYS=Simulated days per arm [1]:
if "%DAYS%"=="" set "DAYS=1"

echo.
echo The two arms are extra command-line arguments for the simulation.
set /p BASELINE_ARGS=Baseline arm args [--maxKnownDarkEdgeCostMultiplier=1.0]:
if "%BASELINE_ARGS%"=="" set "BASELINE_ARGS=--maxKnownDarkEdgeCostMultiplier=1.0"

set /p TREATMENT_ARGS=Treatment arm args ^(blank = model defaults^):

echo.
echo Comparing on "%CITY_NAME%": baseline "%BASELINE_ARGS%" against treatment "%TREATMENT_ARGS%"

python "%ROOT%scripts\run_day_night_comparison.py" --city "%CITY_NAME%" --percentage %PERCENTAGE% --days %DAYS% --baseline-args "%BASELINE_ARGS%" --treatment-args "%TREATMENT_ARGS%"
set "EXITCODE=%ERRORLEVEL%"
pause
exit /b %EXITCODE%
