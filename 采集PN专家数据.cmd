@echo off
setlocal
cd /d "%~dp0"
set "DART_PYTHON=%~dp0guidance_sim\.venv\Scripts\python.exe"
if not exist "%DART_PYTHON%" (
    echo Python environment was not found: %DART_PYTHON%
    pause
    exit /b 1
)
"%DART_PYTHON%" -B -X utf8 "%~dp0guidance_sim\learning\dataset.py" %*
if errorlevel 1 (
    echo Dataset collection failed. Existing results were retained for diagnosis.
    pause
    exit /b 1
)
echo Dataset collection complete. Read the output path and report above.
pause
endlocal
