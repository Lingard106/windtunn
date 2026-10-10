@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
cd /d %~dp0..\fluidx3d
cl.exe /nologo /O2 /EHsc /std:c++17 /W0 /DGRAPHICS=0 /I src\OpenCL\include src\graphics.cpp src\info.cpp src\kernel.cpp src\lbm.cpp src\lodepng.cpp src\main.cpp src\setup.cpp src\shapes.cpp /link /LIBPATH:src\OpenCL\lib OpenCL.lib /OUT:fluidx3d.exe
echo EXITCODE=%ERRORLEVEL%
