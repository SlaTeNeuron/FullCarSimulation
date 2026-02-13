@echo off
REM Build and run the debug test executable
REM This allows testing the simulation without Unity

echo ========================================
echo Building Racing Simulation Debug Test
echo ========================================

cd /d "%~dp0"

REM Create build directory if it doesn't exist
if not exist build mkdir build
cd build

REM Configure with CMake
echo Configuring CMake...
cmake ..
if errorlevel 1 (
    echo ERROR: CMake configuration failed
    pause
    exit /b 1
)

REM Build the project
echo.
echo Building project...
cmake --build . --config Debug
if errorlevel 1 (
    echo ERROR: Build failed
    pause
    exit /b 1
)

echo.
echo ========================================
echo Build successful!
echo ========================================
echo Debug executable location: ..\build\Debug\sim_debug.exe
echo.

REM Ask if user wants to run it
set /p run="Run debug test now? (y/n): "
if /i "%run%"=="y" (
    echo.
    echo Running debug test...
    echo.
    cd ..
    if exist "build\Debug\sim_debug.exe" (
        .\build\Debug\sim_debug.exe
    ) else (
        echo ERROR: sim_debug.exe not found at build\Debug\sim_debug.exe
        echo Build may have failed.
    )
    cd unityInterface\build
)

pause
