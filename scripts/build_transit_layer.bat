@echo off
setlocal EnableExtensions

REM PedSimCity transit-layer launcher.
REM Snaps a GTFS feed's stops to the city's pedestrian street nodes and writes
REM transit_stops.gpkg / transit_stops.csv into src\main\resources\<City>.
REM Reads src\main\resources\<City>\<City>_nodes.gpkg and <City>_gtfs\.
REM No py.exe. No user-specific hardcoded paths. Uses Conda + environment.yml.
REM Prompts for the city name - no city is named in the Python script.

rem This script lives in scripts/; ROOT is the repo root one level up.
set "ROOT=%~dp0..\"
set "ENV_NAME=pedsimcity"
set "ENV_FILE=%ROOT%environment.yml"
set "PROJECT_CONDARC=%ROOT%.condarc"

REM Keep Conda channel config local to this launcher (see build_lighting.bat).
set "CONDARC=%PROJECT_CONDARC%"
set "CONDA_CHANNELS=conda-forge"
set "CONDA_CHANNEL_PRIORITY=strict"

call :find_conda
if errorlevel 1 (
    echo Conda not found.
    echo Install Miniconda/Anaconda, or add conda.exe to PATH.
    pause
    exit /b 1
)

echo Using Conda: "%CONDA_EXE%"

if not exist "%ENV_FILE%" (
    echo Missing environment.yml at:
    echo "%ENV_FILE%"
    pause
    exit /b 1
)

if not exist "%PROJECT_CONDARC%" (
    echo Missing project Conda config at:
    echo "%PROJECT_CONDARC%"
    pause
    exit /b 1
)

call :ensure_env
if errorlevel 1 (
    echo Failed to create/update Conda environment: %ENV_NAME%
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

echo Building transit layer for: "%CITY_NAME%"

"%CONDA_EXE%" run --no-capture-output -n "%ENV_NAME%" python "%ROOT%scripts\build_transit_layer.py" --city "%CITY_NAME%"
set "EXITCODE=%ERRORLEVEL%"
pause
exit /b %EXITCODE%

:find_conda
if defined CONDA_EXE (
    if exist "%CONDA_EXE%" exit /b 0
)

for /f "delims=" %%I in ('where conda.exe 2^>nul') do (
    set "CONDA_EXE=%%I"
    exit /b 0
)

REM Windows paths are case-insensitive, so lowercase entries also match Miniconda3/Anaconda3.
for %%D in (
    "%LOCALAPPDATA%\miniconda3"
    "%LOCALAPPDATA%\anaconda3"
    "%USERPROFILE%\miniconda3"
    "%USERPROFILE%\anaconda3"
    "C:\ProgramData\miniconda3"
    "C:\ProgramData\anaconda3"
) do (
    if exist "%%~D\Scripts\conda.exe" (
        set "CONDA_EXE=%%~D\Scripts\conda.exe"
        exit /b 0
    )
)

exit /b 1

:ensure_env
"%CONDA_EXE%" env list | findstr /B /C:"%ENV_NAME% " >nul 2>nul
if not errorlevel 1 (
    echo Conda environment already exists: %ENV_NAME% ^(reusing as-is^)
    exit /b 0
)

echo Creating Conda environment from environment.yml: %ENV_NAME%
"%CONDA_EXE%" env create -f "%ENV_FILE%"

REM Verify after creation to avoid false failures from Conda transaction edge cases.
"%CONDA_EXE%" env list | findstr /B /C:"%ENV_NAME% " >nul 2>nul
if not errorlevel 1 (
    echo Conda environment created: %ENV_NAME%
    exit /b 0
)

exit /b 1
