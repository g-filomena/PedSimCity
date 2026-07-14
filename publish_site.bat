@echo off
setlocal EnableExtensions

REM Publishes the simulation result pages (outputs\results\*.html) to Cloudflare Pages
REM via publish_site.py (standard-library Python only). One-time setup: install
REM wrangler (npm install -g wrangler), wrangler login, create the Pages project and
REM attach the custom domain — see README "Publishing results".

set "ROOT=%~dp0"

REM publish_site.py needs no GIS packages: any Python 3 will do.
where python >nul 2>nul
if not errorlevel 1 (
    python "%ROOT%publish_site.py" %*
    set "EXITCODE=%ERRORLEVEL%"
    pause
    exit /b %EXITCODE%
)

REM Fall back to the pedsimcity Conda environment.
for /f "delims=" %%I in ('where conda.exe 2^>nul') do set "CONDA_EXE=%%I"
if not defined CONDA_EXE (
    if exist "%LOCALAPPDATA%\miniconda3\Scripts\conda.exe" set "CONDA_EXE=%LOCALAPPDATA%\miniconda3\Scripts\conda.exe"
)
if not defined CONDA_EXE (
    echo No Python found. Install Python or Miniconda.
    pause
    exit /b 1
)
"%CONDA_EXE%" run --no-capture-output -n pedsimcity python "%ROOT%publish_site.py" %*
set "EXITCODE=%ERRORLEVEL%"
pause
exit /b %EXITCODE%
