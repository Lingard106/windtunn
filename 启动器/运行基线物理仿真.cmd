@echo off
setlocal
for %%I in ("%~dp0..") do set "DART_WORKSPACE=%%~fI"
cd /d "%DART_WORKSPACE%"
set "DART_PYTHON=%DART_WORKSPACE%\guidance_sim\.venv\Scripts\python.exe"
if not exist "%DART_PYTHON%" (
    echo Python environment was not found:
    echo %DART_PYTHON%
    pause
    exit /b 1
)
"%DART_PYTHON%" "%DART_WORKSPACE%\guidance_sim\prototypes\dart_viewer.py" --season 2027 --flight-control on --flight-config "%DART_WORKSPACE%\guidance_sim\profiles\flight_control_v1_direct_2027.json" --auto-run %*
if errorlevel 1 (
    echo The viewer exited with an error. Details are shown above.
    pause
    exit /b 1
)
endlocal
