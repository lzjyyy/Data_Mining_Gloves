@echo off
setlocal
cd /d "%~dp0"

set "MOSQUITTO_EXE=C:\Program Files\mosquitto\mosquitto.exe"
if not exist "%MOSQUITTO_EXE%" set "MOSQUITTO_EXE=C:\Program Files (x86)\mosquitto\mosquitto.exe"
if not exist "%MOSQUITTO_EXE%" set "MOSQUITTO_EXE=mosquitto"

"%MOSQUITTO_EXE%" -c mosquitto_windows.conf -v
