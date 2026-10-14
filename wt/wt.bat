@echo off
rem wt.bat -- double-click / cmd shim for wt\wt.sh (Git Bash required)
setlocal
set "SH=%~dp0wt.sh"
set "SHFWD=%SH:\=/%"
where bash >nul 2>nul
if %errorlevel%==0 (
  bash "%SHFWD%" %*
  goto :end
)
if exist "%ProgramFiles%\Git\bin\bash.exe" (
  "%ProgramFiles%\Git\bin\bash.exe" "%SHFWD%" %*
  goto :end
)
echo wt: bash not found -- install Git for Windows
exit /b 1
:end
endlocal
