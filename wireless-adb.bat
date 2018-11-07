@echo off
pushd %LOCALAPPDATA%\Android\Sdk\platform-tools

if "%1"=="enable" goto enable
if "%1"=="disable" goto disable

goto Usage

:enable
echo Enabling Wireless ADB
adb tcpip 5555
REM adb devices
REM adb shell ifconfig
adb enable 192.168.49.1:5555
goto Done

:disable
echo Disabling Wireless ADB
adb disable
goto Done

:Usage
echo Usage: %0 "<enable|disable>"

:Done
popd
