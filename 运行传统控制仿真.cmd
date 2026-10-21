@echo off
setlocal
cd /d "%~dp0"
set "DART_PYTHON=%~dp0guidance_sim\.venv\Scripts\python.exe"
if not exist "%DART_PYTHON%" (
    echo Python environment was not found: %DART_PYTHON%
    pause
    exit /b 1
)
"%DART_PYTHON%" "%~dp0guidance_sim\traditional_control\run.py" --auto-run %*
if errorlevel 1 (
    echo The physical traditional-control viewer exited with an error.
    pause
    exit /b 1
)
endlocal
