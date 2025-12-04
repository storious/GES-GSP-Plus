@echo off
>nul chcp 65001

setlocal enabledelayedexpansion

REM =================================================================
REM GES-GSP-Plus Build and Run Script
REM =================================================================
REM Usage:
REM   build_and_run.bat [preset] [type] [target]
REM   build_and_run.bat [preset] [type] -file [file_list]
REM   build_and_run.bat -h
REM
REM Examples:
REM   build_and_run.bat                     (Default: vcpkg debug, run RUN_FILE.txt)
REM   build_and_run.bat vcpkg release       (Build with vcpkg-release, run RUN_FILE.txt)
REM   build_and_run.bat conan debug my_data (Build with conan-debug, run 'my_data')
REM   build_and_run.bat vcpkg release -file my_list.txt (Build and run from 'my_list.txt')
REM =================================================================

REM --- 0. Check for help argument first ---
if /i "%~1"=="-h" goto :show_help
if /i "%~1"=="--help" goto :show_help

REM --- 1. Define Configuration ---
set VALID_PRESETS=vcpkg conan
set VALID_TYPES=debug release
set DEFAULT_PRESET=vcpkg
set DEFAULT_TYPE=debug

REM --- 2. Parse Arguments ---
set PRESET_NAME=%DEFAULT_PRESET%
set PRESET_TYPE=%DEFAULT_TYPE%
set SINGLE_TARGET=
set FILE_LIST_PATH=
set TARGET_SPECIFIED=0

:parse_loop
if "%~1"=="" goto parse_done

if /i "%~1"=="vcpkg" (
    set PRESET_NAME=%~1
) else if /i "%~1"=="conan" (
    set PRESET_NAME=%~1
) else if /i "%~1"=="debug" (
    set PRESET_TYPE=%~1
) else if /i "%~1"=="release" (
    set PRESET_TYPE=%~1
) else if /i "%~1"=="-file" (
    shift
    set FILE_LIST_PATH=%~1
    set TARGET_SPECIFIED=1
) else (
    REM Treat any other argument as a single target input
    set SINGLE_TARGET=%~1
    set TARGET_SPECIFIED=1
)
shift
goto parse_loop

:parse_done

REM --- 3. Validate Build Arguments ---
set IS_VALID_NAME=0
for %%P in (%VALID_PRESETS%) do if /i "!PRESET_NAME!"=="%%P" set IS_VALID_NAME=1
if !IS_VALID_NAME!==0 (
    echo [ERROR] Invalid preset name: "!PRESET_NAME!"
    echo Valid options are: vcpkg, conan
    goto :eof
)

set IS_VALID_TYPE=0
for %%T in (%VALID_TYPES%) do if /i "!PRESET_TYPE!"=="%%T" set IS_VALID_TYPE=1
if !IS_VALID_TYPE!==0 (
    echo [ERROR] Invalid build type: "!PRESET_TYPE!"
    echo Valid options are: debug, release
    goto :eof
)

REM --- 4. Build Project ---
echo.
echo [INFO] Starting build with preset: !PRESET_NAME!-!PRESET_TYPE!
echo.
cd ./Code
cmake --build --preset !PRESET_NAME!-!PRESET_TYPE!
if !ERRORLEVEL! neq 0 (
    echo.
    echo [ERROR] CMake build failed. Please check the output above for errors.
    cd ..
    exit /b 1
)
cd ..

REM --- 5. Determine Executable and Target List ---
set EXE_PATH=%~dp0Code\build\!PRESET_TYPE!\ges_stitching.exe

if not exist "!EXE_PATH!" (
    echo [ERROR] Executable not found: !EXE_PATH!
    echo Make sure the build completed successfully.
    pause
    exit /b 1
)

REM --- FIX: Logic to determine what to run ---
set RUN_MODE=
set RUN_TARGET=

if !TARGET_SPECIFIED!==1 (
    if defined FILE_LIST_PATH (
        set RUN_MODE=list
        set RUN_TARGET=!FILE_LIST_PATH!
    ) else if defined SINGLE_TARGET (
        set RUN_MODE=single
        set RUN_TARGET=!SINGLE_TARGET!
    )
) else (
    REM Default behavior: run the default file list
    set RUN_MODE=list
    set RUN_TARGET=%~dp0RUN_FILE.txt
)

if not exist "!RUN_TARGET!" (
    echo [ERROR] Target not found: !RUN_TARGET!
    pause
    exit /b 1
)

REM --- 6. Run Application ---
echo.
echo [INFO] Build successful. Launching application...
echo [INFO] Running in mode: !RUN_MODE!
echo [INFO] Using target: !RUN_TARGET!
echo ------------------------------------------------------------

if "!RUN_MODE!"=="list" (
    REM Run from file list
    for /f "usebackq delims=" %%i in ("!RUN_TARGET!") do (
        set target=%%i
        if "!target!"=="" (
            echo Skipping empty line...
        ) else (
            echo -----------------------------------------------------------------
            echo Start !target!
            "!EXE_PATH!" "!target!"
            echo Finish !target!
            echo -----------------------------------------------------------------
        )
    )
) else (
    REM Run single target
    echo -----------------------------------------------------------------
    echo Start !RUN_TARGET!
    "!EXE_PATH!" "!RUN_TARGET!"
    echo Finish !RUN_TARGET!
    echo -----------------------------------------------------------------
)

echo.
echo All tasks finished!
goto :eof

REM =================================================================
REM Help Section (Revised for better encoding compatibility)
REM =================================================================
:show_help
echo.
echo GES-GSP-Plus Build and Run Script
echo.
echo DESCRIPTION:
echo   This script automates the build process for GES-GSP-Plus using CMake presets
echo   and then runs the resulting executable on a specified target.
echo.
echo USAGE:
echo   %~nx0 [preset] [type] [target]
echo   %~nx0 [preset] [type] -file [file_list]
echo   %~nx0 -h
echo.
echo PARAMETERS:
echo   preset        (Optional) The dependency manager to use. Default: vcpkg
echo                  Values: vcpkg, conan
echo.
echo   type          (Optional) The build configuration. Default: debug
echo                  Values: debug, release
echo.
echo   target        (Optional) If specified, runs the application on this single
echo                  input. Overrides the default RUN_FILE.txt.
echo.
echo   -file ^<path^> (Optional) Specifies a file containing a list of inputs.
echo                  Overrides the default RUN_FILE.txt.
echo.
echo   -h, --help    Displays this help message.
echo.
echo EXAMPLES:
echo   %~nx0
echo   Builds with 'vcpkg-debug' preset and runs inputs from RUN_FILE.txt.
echo.
echo   %~nx0 conan release
echo   Builds with 'conan-release' preset and runs inputs from RUN_FILE.txt.
echo.
echo   %~nx0 vcpkg debug my_dataset
echo   Builds with 'vcpkg-debug' preset and runs on the single target 'my_dataset'.
echo.
echo   %~nx0 vcpkg release -file C:\data\my_list.txt
echo   Builds with 'vcpkg-release' preset and runs all inputs from the specified list.
echo.
goto :eof
