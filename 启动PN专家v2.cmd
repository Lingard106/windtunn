@echo off
setlocal
cd /d "%~dp0"
set "DART_PYTHON=%~dp0guidance_sim\.venv\Scripts\python.exe"
if not exist "%DART_PYTHON%" (
    echo Python environment was not found: %DART_PYTHON%
    pause
    exit /b 1
)
"%DART_PYTHON%" "%~dp0guidance_sim\prototypes\ideal_guidance_sim.py" --flight-config "%~dp0guidance_sim\pn_expert_v2\config.json" --guidance-source vision --output "%~dp0guidance_sim\outputs\pn_expert_v2" --auto-run %*
if errorlevel 1 (
    echo The PN expert v2 viewer exited with an error. Details are shown above.
    pause
    exit /b 1
)
endlocal
