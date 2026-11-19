@echo off
set "WEBOTS=%ProgramFiles%\Webots\msys64\mingw64\bin\webotsw.exe"
if not exist "%WEBOTS%" (
  echo Webots не найден. Установите Webots R2025a.
  echo Ожидался файл:
  echo "%WEBOTS%"
  pause
  exit /b 1
)
if not exist "%~dp0worlds\mavic_manual.wbt" (
  echo Не найден мир worlds\mavic_manual.wbt
  echo Распакуйте архив целиком и запускайте этот файл из папки mavic_patrol.
  pause
  exit /b 1
)
start "" "%WEBOTS%" "%~dp0worlds\mavic_manual.wbt"
