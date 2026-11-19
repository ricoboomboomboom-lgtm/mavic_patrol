@echo off
set "WEBOTS=%ProgramFiles%\Webots\msys64\mingw64\bin\webotsw.exe"
if not exist "%WEBOTS%" (
  echo Webots R2025a не найден: "%WEBOTS%"
  pause
  exit /b 1
)
start "" "%WEBOTS%" "%~dp0worlds\mavic_patrol.wbt"
