@echo off
setlocal

set PRESET_NAME=%1

set PRESET_TYPE=%2

:: Check if a parameter was provided
if %PRESET_NAME%=="" (
    set PRESET_NAME="vcpkg"
)

if %PRESET_TYPE%=="" (
    set PRESET_TYPE="debug"
)

cd Code
cmake --build --preset %PRESET_NAME%-%PRESET_TYPE%

:: Check if build was successful
if errorlevel 1 (
    echo ERROR: Project build failed. Aborting execution.
    echo ----------------------------------------
    goto :eof
)
echo Build completed successfully.

cd ..
echo Running executable via RUN_EXE.bat...
.\RUN_EXE.bat

endlocal