@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
cd /d %~dp0fluidx3d
copy /y %~dp0setup_windtunnel_fan.cpp src\setup_windtunnel_fan.cpp >nul
copy /y %~dp0windtunnel.raw windtunnel.raw >nul
copy /y %~dp0fan_blade.bin fan_blade.bin >nul
copy /y %~dp0fan_rotor.bin fan_rotor.bin >nul
cl.exe /nologo /O2 /EHsc /std:c++17 /W0 /utf-8 /DGRAPHICS=0 /DFORCE_FIELD /DMOVING_BOUNDARIES /I src\OpenCL\include src\graphics.cpp src\info.cpp src\kernel.cpp src\lbm.cpp src\lodepng.cpp src\main.cpp src\setup_windtunnel_fan.cpp src\shapes.cpp /link /LIBPATH:src\OpenCL\lib OpenCL.lib /OUT:windtunnel_fan.exe
echo BUILD_EXITCODE=%ERRORLEVEL%
