@echo off
chcp 65001 >nul
cd /d "%~dp0"
set "WEBOTS=%ProgramFiles%\Webots\msys64\mingw64\bin\webotsw.exe"
if not exist "%WEBOTS%" (
  echo Webots не найден. Установите Webots R2025a.
  echo Ожидался файл:
  echo "%WEBOTS%"
  pause
  exit /b 1
)
if not exist "%~dp0controllers\mavic_manual\mavic_manual.exe" (
  echo Не найден контроллер пульта controllers\mavic_manual\mavic_manual.exe
  pause
  exit /b 1
)
if not exist "%~dp0controllers\mavic2pro_controller\mavic2pro_controller.exe" (
  echo Не найден контроллер патруля controllers\mavic2pro_controller\mavic2pro_controller.exe
  pause
  exit /b 1
)

echo.
echo  1  Арена
echo  2  Холмы и лес
echo  3  Город
echo  4  Набережная
echo  5  Река и мост
echo  6  Посёлок
echo  7  Порт
echo  8  Ущелье
echo  9  Поля
echo  0  Остров
echo.
choice /c 1234567890 /n /m "Карта: "
if errorlevel 10 goto map_island
if errorlevel 9 goto map_fields
if errorlevel 8 goto map_canyon
if errorlevel 7 goto map_port
if errorlevel 6 goto map_village
if errorlevel 5 goto map_river
if errorlevel 4 goto map_shore
if errorlevel 3 goto map_city
if errorlevel 2 goto map_terrain
goto map_arena

:map_arena
set "WORLD=worlds\mavic_arena.wbt"
goto pick_controller
:map_terrain
set "WORLD=worlds\mavic_terrain.wbt"
goto pick_controller
:map_city
set "WORLD=worlds\mavic_city.wbt"
goto pick_controller
:map_shore
set "WORLD=worlds\mavic_shore.wbt"
goto pick_controller
:map_river
set "WORLD=worlds\mavic_river.wbt"
goto pick_controller
:map_village
set "WORLD=worlds\mavic_village.wbt"
goto pick_controller
:map_port
set "WORLD=worlds\mavic_port.wbt"
goto pick_controller
:map_canyon
set "WORLD=worlds\mavic_canyon.wbt"
goto pick_controller
:map_fields
set "WORLD=worlds\mavic_fields.wbt"
goto pick_controller
:map_island
set "WORLD=worlds\mavic_island.wbt"
goto pick_controller

:pick_controller
echo.
echo  1  Пульт TX12
echo  2  Патруль
echo.
choice /c 12 /n /m "Контроллер: "
if errorlevel 2 goto use_patrol
set "CTRL=mavic_manual"
goto launch
:use_patrol
set "CTRL=mavic2pro_controller"
goto launch

:launch
if not exist "%~dp0%WORLD%" (
  echo Не найден мир %WORLD%
  pause
  exit /b 1
)
if "%CTRL%"=="mavic2pro_controller" (
  powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0_select.ps1" -World "%~dp0%WORLD%" -Controller "%CTRL%" -Mission "area" -Out "%~dp0worlds\_selected.wbt"
) else (
  powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0_select.ps1" -World "%~dp0%WORLD%" -Controller "%CTRL%" -Out "%~dp0worlds\_selected.wbt"
)
if errorlevel 1 (
  echo Не удалось подготовить мир.
  pause
  exit /b 1
)
start "" "%WEBOTS%" --mode=realtime "%~dp0worlds\_selected.wbt"
exit /b 0
