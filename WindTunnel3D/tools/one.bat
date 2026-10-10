@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
cd /d %~dp0..\fluidx3d
cl.exe /nologo /O2 /EHsc /std:c++17 /W0 /DGRAPHICS=0 /DFORCE_FIELD /I src\OpenCL\include /c /Fo:swt.obj src\setup_windtunnel.cpp
echo ONE_EXIT=%ERRORLEVEL%
