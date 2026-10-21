@echo off
setlocal
for %%I in ("%~dp0..") do set "DART_WORKSPACE=%%~fI"
cd /d "%DART_WORKSPACE%"
set "DART_PYTHON=%DART_WORKSPACE%\guidance_sim\.venv\Scripts\python.exe"
if not exist "%DART_PYTHON%" (
    echo Python environment was not found: %DART_PYTHON%
    pause
    exit /b 1
)
"%DART_PYTHON%" "%DART_WORKSPACE%\guidance_sim\prototypes\ideal_guidance_sim.py" --flight-config "%DART_WORKSPACE%\guidance_sim\pn_expert_v2\config.json" --guidance-source vision --auto-run %*
if errorlevel 1 (
    echo The PN expert v2 viewer exited with an error. Details are shown above.
    pause
    exit /b 1
)
endlocal
