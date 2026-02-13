@echo off
REM Quick run script for the debug executable
REM Assumes it has already been built

cd /d "%~dp0"

if not exist "..\build\Debug\sim_debug.exe" (
    echo ERROR: Debug executable not found!
    echo Please run build_debug.bat first to build the project.
    pause
    exit /b 1
)

echo Running Racing Simulation Debug Test...
echo.

REM Parse command line arguments and pass them through
cd ..
.\build\Debug\sim_debug.exe %*
cd unityInterface

echo.
pause
