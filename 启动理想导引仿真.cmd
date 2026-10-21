@echo off
setlocal
cd /d "%~dp0"
set "DART_PYTHON=%~dp0guidance_sim\.venv\Scripts\python.exe"
if not exist "%DART_PYTHON%" (
    echo Python environment was not found:
    echo %DART_PYTHON%
    pause
    exit /b 1
)
"%DART_PYTHON%" "%~dp0guidance_sim\prototypes\ideal_guidance_sim.py" --guidance-source vision --auto-run %*
if errorlevel 1 (
    echo The ideal guidance viewer exited with an error. Details are shown above.
    pause
    exit /b 1
)
endlocal
