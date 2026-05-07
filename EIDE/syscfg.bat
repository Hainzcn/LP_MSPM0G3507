@echo off

set "SYSCFG_PATH=A:\Program Files\ti\sysconfig_1.27.0\sysconfig_cli.bat"
set "SDK_ROOT=A:\ti\mspm0_sdk_2_10_00_04"

if not exist "%SYSCFG_PATH%" (
    echo.
    echo Couldn't find Sysconfig Tool "%SYSCFG_PATH%"
    echo "Update the file located at ^<sdk path^>/tools/keil/syscfg.bat"
    echo.
    exit /b 1
)

echo Using Sysconfig Tool from "%SYSCFG_PATH%"
echo Using MSPM0 SDK from "%SDK_ROOT%"

set PROJ_DIR=%~1
set PROJ_DIR=%PROJ_DIR:'=%

set SYSCFG_FILE=%~2
set SYSCFG_FILE=%SYSCFG_FILE:'=%

if not exist "%SDK_ROOT%\.metadata\product.json" (
    echo.
    echo Couldn't find SDK product metadata "%SDK_ROOT%\.metadata\product.json"
    echo Update SDK_ROOT in this file if the MSPM0 SDK is installed elsewhere.
    echo.
    exit /b 1
)

:: Search for the directory containing the requested syscfg file.
:: Go up at least 5 times, then give up.
set SYSCFG_DIR=%PROJ_DIR%
set iter=0
:syscfg_search_loop
if exist "%SYSCFG_DIR%\%SYSCFG_FILE%" (
    rem Remove the trailing slash if it exists since Keil doesn't like it.
    if "%SYSCFG_DIR:~-1%"=="\" set "SYSCFG_DIR=%SYSCFG_DIR:~0,-1%"
    goto syscfg_search_exit
) else if %iter% geq 5 (
	echo Couldn't find syscfg file "%SYSCFG_FILE%"
	exit /b 1
) else (
	set /a iter=%iter%+1
	set SYSCFG_DIR=%SYSCFG_DIR%..\
	goto syscfg_search_loop
)
:syscfg_search_exit

call "%SYSCFG_PATH%" -o "%SYSCFG_DIR%" -s "%SDK_ROOT%\.metadata\product.json" --compiler keil "%SYSCFG_DIR%\%SYSCFG_FILE%"
