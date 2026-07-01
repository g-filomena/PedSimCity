@echo off
setlocal EnableExtensions

REM PedSimCity street-lighting launcher - Turin `puntiLuce` inventory.
REM No py.exe. No user-specific hardcoded paths. Uses Conda + environment.yml.

set "ROOT=%~dp0"
set "ENV_NAME=pedsimcity"
set "ENV_FILE=%ROOT%environment.yml"

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

call :ensure_env
if errorlevel 1 (
    echo Failed to create Conda environment: %ENV_NAME%
    pause
    exit /b 1
)

set /p CITY=Enter city name ^(folder under src\main\resources^):
if "%CITY%"=="" (
    echo No city name entered. Exiting.
    pause
    exit /b 1
)

set "INPUT_DIR=%ROOT%src\main\resources\%CITY%"

if not exist "%INPUT_DIR%" (
    echo Input directory not found:
    echo "%INPUT_DIR%"
    pause
    exit /b 1
)

echo Building street lighting (Turin): "%INPUT_DIR%"

"%CONDA_EXE%" run --no-capture-output -n "%ENV_NAME%" python "%ROOT%pipeline\build_lighting_torino.py" --input_dir "%INPUT_DIR%"
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
