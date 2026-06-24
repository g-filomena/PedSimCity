@echo off
setlocal
REM Builds the census step only (residence + workplace + night POI + vulnerability)
REM into one <City>_censusData.gpkg.

set /p CITY=Enter city name (folder under src\main\resources):
if "%CITY%"=="" (
    echo No city name entered. Exiting.
    pause
    exit /b 1
)

set /p PREFIX=Enter raw-file prefix (e.g. Torino_):

set "INPUT_DIR=%~dp0src\main\resources\%CITY%"
echo Building census data: "%INPUT_DIR%"  (prefix "%PREFIX%")
python "%~dp0pipeline\01_census_and_poi.py" --input_dir "%INPUT_DIR%" --prefix "%PREFIX%"
pause
