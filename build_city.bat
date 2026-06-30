@echo off
setlocal
REM Builds the full city data pipeline:
REM census + POI + vulnerability, then street lighting (steps 1-4).

set /p CITY=Enter city name (folder under src\main\resources):
if "%CITY%"=="" (
    echo No city name entered. Exiting.
    pause
    exit /b 1
)

set /p PREFIX=Enter raw-file prefix (e.g. Torino_):

set "INPUT_DIR=%~dp0src\main\resources\%CITY%"
echo Building city data: "%INPUT_DIR%"  (prefix "%PREFIX%")

py -3 "%~dp0pipeline\build_city.py" --input_dir "%INPUT_DIR%" --prefix "%PREFIX%"

pause