@echo off
setlocal EnableExtensions

REM PedSimCity city-preparation launcher.
REM Builds the base simulation layers (network, dual graph, districts, barriers,
REM landmarks, sight lines) for a city from OSM via cityImage, replacing the
REM per-city PedSimCity_Preparation notebooks.
REM No py.exe. No user-specific hardcoded paths. Uses Conda + environment-prep.yml.
REM
REM Prompts for the resources folder name, then for a STAGE GROUP:
REM   [1] everything
REM   [2] base layers (buildings, network, districts, elevation, barriers, pois) - fast, inspect in QGIS
REM   [3] landmarks (buildings, network, elevation, sightlines, landmarks) - heavy, run overnight
REM   [4] custom stage list
REM Buildings runs first (defines the network clip extent); stages execute in pipeline order, so a
REM group only needs to include its prerequisites - already-checkpointed stages are skipped.
REM After the stage choice it also asks whether to consolidate network nodes (merge nearby graph
REM nodes in the network stage) and, when yes, the consolidation tolerance in metres.
REM The OSM place query and EPSG are asked only on the FIRST run for a city; they are
REM saved to <City>\prep_config.json and reused, so stage re-runs cannot mismatch them.
REM Stages checkpoint under <City>\prep_staging\ - re-running resumes automatically.

set "ROOT=%~dp0"
set "ENV_NAME=pedsimcity-prep"
set "ENV_FILE=%ROOT%environment-prep.yml"
set "PROJECT_CONDARC=%ROOT%.condarc"

REM Keep Conda channel config local to this launcher (see build_census.bat).
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
    echo Missing environment-prep.yml next to this .bat:
    echo "%ENV_FILE%"
    pause
    exit /b 1
)

if not exist "%PROJECT_CONDARC%" (
    echo Missing project Conda config next to this .bat:
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

call :install_cityimage
if errorlevel 1 (
    echo Failed to install cityImage into %ENV_NAME%.
    pause
    exit /b 1
)

set /p FOLDER_NAME=Enter city name ^(e.g. Torino^):
if "%FOLDER_NAME%"=="" (
    echo No city name entered. Exiting.
    pause
    exit /b 1
)

REM Raw inputs are read from inputData\<City>; outputs land in src\main\resources\<City>.
REM Place/EPSG: asked once per city, then read back from prep_config.json by the script.
set "CITY_ARGS="
set "CONSOLIDATE_ARGS="
if exist "%ROOT%inputData\%FOLDER_NAME%\prep_config.json" (
    echo Using saved place/EPSG from "%ROOT%inputData\%FOLDER_NAME%\prep_config.json".
    goto :choose_stages
)

set /p PLACE_NAME=Enter OSM place query ^(e.g. Torino, Italy^):
if "%PLACE_NAME%"=="" (
    echo No place entered. Exiting.
    pause
    exit /b 1
)

set /p EPSG_CODE=Enter projected EPSG code ^(e.g. 3003^):
if "%EPSG_CODE%"=="" (
    echo No EPSG code entered. Exiting.
    pause
    exit /b 1
)
set CITY_ARGS=--place "%PLACE_NAME%" --epsg %EPSG_CODE%

:choose_stages
echo.
echo Which stages?
echo   [1] everything (default)
echo   [2] base layers: buildings, network, districts, elevation, barriers, pois   (fast - inspect in QGIS)
echo   [3] landmarks: buildings, network, elevation, sightlines, landmarks  (heavy - run overnight)
echo   [4] custom stage list
echo   (buildings runs first and defines the extent the network is clipped to; stages always run
echo    in pipeline order, so a group just needs to include its prerequisites - already-done ones skip.)
set /p STAGE_CHOICE=Choice [1]:
set "STAGE_ARGS="
if "%STAGE_CHOICE%"=="2" set "STAGE_ARGS=--stages buildings,network,districts,elevation,barriers,pois"
if "%STAGE_CHOICE%"=="3" set "STAGE_ARGS=--stages buildings,network,elevation,sightlines,landmarks"
if not "%STAGE_CHOICE%"=="4" goto :consolidation

set /p CUSTOM_STAGES=Enter stages ^(comma-separated: buildings,network,districts,elevation,barriers,pois,sightlines,landmarks^):
if "%CUSTOM_STAGES%"=="" (
    echo No stages entered. Exiting.
    pause
    exit /b 1
)
set "STAGE_ARGS=--stages %CUSTOM_STAGES%"

:consolidation
echo.
echo Network node consolidation (merges nearby graph nodes; affects the network stage).
set /p CONSOLIDATE=Consolidate network nodes? yes/no [yes]:
if "%CONSOLIDATE%"=="" set "CONSOLIDATE=yes"
set "CONSOLIDATE_ARGS=--consolidate-network %CONSOLIDATE%"
if /I "%CONSOLIDATE%"=="no" goto :run_pipeline
if /I "%CONSOLIDATE%"=="n" goto :run_pipeline
set /p CONS_TOL=Consolidation tolerance in metres [15]:
if "%CONS_TOL%"=="" set "CONS_TOL=15"
set "CONSOLIDATE_ARGS=%CONSOLIDATE_ARGS% --consolidate-tolerance %CONS_TOL%"

:run_pipeline
echo Preparing city layers for: "%FOLDER_NAME%"
echo (Stages are checkpointed under inputData\%FOLDER_NAME%\prep_staging\ - re-running resumes where it stopped.)

"%CONDA_EXE%" run --no-capture-output -n "%ENV_NAME%" python "%ROOT%pipeline\00_city_preparation.py" --city "%FOLDER_NAME%" %CITY_ARGS% %STAGE_ARGS% %CONSOLIDATE_ARGS%
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
    echo   To rebuild it from environment-prep.yml, delete it first:
    echo   "%CONDA_EXE%" env remove -n %ENV_NAME%
    exit /b 0
)

echo Creating Conda environment from environment-prep.yml: %ENV_NAME%
"%CONDA_EXE%" env create -f "%ENV_FILE%"

REM Verify after creation to avoid false failures from Conda transaction edge cases.
"%CONDA_EXE%" env list | findstr /B /C:"%ENV_NAME% " >nul 2>nul
if not errorlevel 1 (
    echo Conda environment created: %ENV_NAME%
    exit /b 0
)

exit /b 1

:install_cityimage
echo Installing cityImage from PyPI.
"%CONDA_EXE%" run --no-capture-output -n "%ENV_NAME%" python -m pip install cityImage
exit /b %ERRORLEVEL%
