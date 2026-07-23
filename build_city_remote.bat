@echo off
setlocal EnableExtensions

REM PedSimCity remote city-preparation launcher.
REM Runs pipeline/00_city_preparation.py on the remote server defined in server.properties
REM (same host/key the Java remote-run uses), inside the pedsimcity-prep conda environment.
REM
REM The heavy lifting (parsing server.properties, prompting, SSH) is in pipeline\remote_prep.ps1;
REM this wrapper just launches it so the tool is double-clickable like build_city.bat.
REM
REM Prep outputs land in the server checkout's src\main\resources\<City>, which is exactly what
REM the Java simulation runs from on the same server - no round-trip needed. Cities that rely on
REM raster (*.tif DTM/DEM) or otherwise-gitignored raw inputs must have those files scp'd up to
REM <projectBaseDir>/PedSimCity/inputData/<City>/ first (git does not carry them).

set "ROOT=%~dp0"

powershell -NoProfile -ExecutionPolicy Bypass -File "%ROOT%pipeline\remote_prep.ps1"
set "EXITCODE=%ERRORLEVEL%"
pause
exit /b %EXITCODE%
