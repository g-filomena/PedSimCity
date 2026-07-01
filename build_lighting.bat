@echo off
setlocal EnableExtensions

REM PedSimCity street-lighting launcher - generic bare lamp-point locations.
REM No py.exe. No user-specific hardcoded paths. Uses Conda + environment.yml.
REM Prompts separately for:
REM   1) resources folder name under src\main\resources
REM   2) city file prefix before _edges.gpkg, _streetlights.gpkg, etc.

set "ROOT=%~dp0"
set "ENV_NAME=pedsimcity"
set "ENV_FILE=%ROOT%environment.yml"
set "PROJECT_CONDARC=%ROOT%.condarc"

REM Avoid Conda warning: "Adding 'defaults' to channel list implicitly is deprecated".
REM Keep Conda channel config local to this launcher and avoid touching user/global .condarc.
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
    echo Missing environment.yml next to this .bat:
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

set /p FOLDER_NAME=Enter resources folder name ^(under src\main\resources^): 
if "%FOLDER_NAME%"=="" (
    echo No folder name entered. Exiting.
    pause
    exit /b 1
)

set /p CITY_NAME=Enter city file prefix ^(before _edges.gpkg / _streetlights.gpkg, e.g. Torino^): 
if "%CITY_NAME%"=="" (
    echo No city file prefix entered. Exiting.
    pause
    exit /b 1
)

set "INPUT_DIR=%ROOT%src\main\resources\%FOLDER_NAME%"

if not exist "%INPUT_DIR%" (
    echo Input directory not found:
    echo "%INPUT_DIR%"
    pause
    exit /b 1
)

if not exist "%INPUT_DIR%\%CITY_NAME%_streetlights.gpkg" (
    echo Expected generic streetlights file not found:
    echo "%INPUT_DIR%\%CITY_NAME%_streetlights.gpkg"
    pause
    exit /b 1
)

if not exist "%INPUT_DIR%\%CITY_NAME%_edges.gpkg" (
    echo Expected edges file not found:
    echo "%INPUT_DIR%\%CITY_NAME%_edges.gpkg"
    pause
    exit /b 1
)

echo Building street lighting ^(generic^) in folder: "%INPUT_DIR%"
echo Using city file prefix: "%CITY_NAME%"

"%CONDA_EXE%" run --no-capture-output -n "%ENV_NAME%" python "%ROOT%pipeline\build_lighting.py" --input_dir "%INPUT_DIR%" --city_name "%CITY_NAME%"
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
    echo Conda environment already exists: %ENV_NAME%
    echo Updating Conda environment from environment.yml...
    "%CONDA_EXE%" env update -n "%ENV_NAME%" -f "%ENV_FILE%" --prune
    exit /b %ERRORLEVEL%
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
