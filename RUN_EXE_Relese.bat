@echo off

REM project root path
set SCRIPT_DIR=%~dp0


REM .exe path
set EXE=%SCRIPT_DIR%/code/build/Release/ges_stitching.exe

REM RUN_FILE.txt path
set FILE_LIST=%SCRIPT_DIR%RUN_FILE.txt

:parse_args
if "%~1"=="" goto args_done

if "%~1"=="-file" (
    shift
    set FILE_LIST=%~1
) else (
    REM other parameter as single input
    set FILE_LIST=%~1
)
shift
goto parse_args

:args_done

if not exist "%EXE%" (
    echo ERROR: Executable not found: %EXE%
    pause
    exit /b 1
)

if not exist "%FILE_LIST%" (
    echo ERROR: File list not found: %FILE_LIST%
    pause
    exit /b 1
)

set PATH=%DLL_DIR%;%PATH%
echo Using file list: %FILE_LIST%
echo ------------------------------------------------------------

setlocal enabledelayedexpansion

for /f "usebackq delims=" %%i in ("%FILE_LIST%") do (
    set target=%%i
    if "!target!"=="" (
        echo Skipping empty line...
    ) else (
        echo -----------------------------------------------------------------
        echo Start !target!
        "%EXE%" "!target!"
        echo Finish !target!
        echo -----------------------------------------------------------------
    )
)

echo All tasks finished!