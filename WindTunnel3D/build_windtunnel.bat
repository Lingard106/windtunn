@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
cd /d D:\WindTunnel3D\fluidx3d
copy /y ..\setup_windtunnel.cpp src\setup_windtunnel.cpp >nul
copy /y ..\windtunnel.raw windtunnel.raw >nul
cl.exe /nologo /O2 /EHsc /std:c++17 /W0 /DGRAPHICS=0 /DFORCE_FIELD /I src\OpenCL\include src\graphics.cpp src\info.cpp src\kernel.cpp src\lbm.cpp src\lodepng.cpp src\main.cpp src\setup_windtunnel.cpp src\shapes.cpp /link /LIBPATH:src\OpenCL\lib OpenCL.lib /OUT:windtunnel.exe
echo BUILD_EXITCODE=%ERRORLEVEL%
