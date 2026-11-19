# Автономный облёт Mavic 2 Pro

Комплект для Webots R2025a. Миссия стартует сама при запуске симуляции: вертикальный взлёт, зигзаг по прямоугольнику впереди дрона, возврат в точку старта и посадка. Пульт не нужен.

Контроллер — один файл на C: `controllers/mavic2pro_controller/mavic2pro_controller.c`.

## Запуск

1. Установите Webots R2025a.
2. Откройте `worlds/mavic_zigzag.wbt`.
3. Если рядом с исходником ещё нет `mavic2pro_controller.exe`, соберите контроллер. В редакторе Webots откройте `mavic2pro_controller.c` и нажмите Build. Либо из PowerShell:

```powershell
$env:WEBOTS_HOME = "C:\Program Files\Webots"
$env:PATH = "C:\Program Files\Webots\msys64\mingw64\bin;C:\Program Files\Webots\msys64\mingw64\bin\cpp;C:\Program Files\Webots\msys64\usr\bin;" + $env:PATH
cd controllers\mavic2pro_controller
make
```

4. Нажмите Play. В консоли раз в секунду идёт телеметрия. В конце успешный проход печатает `Mission completed successfully.`

Каталоги `worlds` и `controllers` должны остаться соседями. Webots берёт контроллер по имени каталога `mavic2pro_controller`.

## Что уже стоит в мире

| Поле | Зачем |
|---|---|
| `controller "mavic2pro_controller"` | Этот контроллер, не штатный пример Webots. |
| `Accelerometer` и `Altimeter` в `bodySlot` | В прототипе Mavic 2 Pro их нет. Без них контроллер останавливает миссию. |
| `basicTimeStep 8` | 125 Гц, как в демонстрационных мирах Mavic. |
| `defaultDamping` 0.5 / 0.5 | Тот же демпфер, под который подобран висений `68.5` рад/с. |
| Арена 60×40 м | Полоса 20 м вперёд и 10 м в ширину начинается в точке старта и идёт по +X. На арене 20×20 дальний край уже за полом. |

GPS, компас, гироскоп и inertial unit берутся из прототипа. Имена моторов: `front left propeller`, `front right propeller`, `rear left propeller`, `rear right propeller`.

## Параметры миссии

В начале `mavic2pro_controller.c`:

```c
#define AREA_LENGTH 20.0
#define AREA_WIDTH 10.0
#define FLIGHT_ALTITUDE 5.0
#define LANE_SPACING 2.0
#define FLIGHT_SPEED 2.0
#define WAYPOINT_TOLERANCE 0.2
```

После правки чисел контроллер нужно пересобрать и заново запустить симуляцию. Точки маршрута считаются из этих величин, списком они не зашиты. Если ширину не делит шаг галсов, шаг уменьшается так, чтобы обе боковые границы вошли в полосу.

Нос при нулевом курсе смотрит на +X, левый борт на +Y. Прямоугольник строится от начального курса компаса: длина вперёд, ширина поперёк, включая ближнюю и дальнюю кромки.

## Как устроен полёт

Конечный автомат: `INITIALIZATION`, `READY`, `TAKEOFF`, `HOVER`, `ZIGZAG_SCAN`, `RETURN_HOME`, `LANDING`, `LANDED`, `EMERGENCY`.

На каждом шаге контроллер читает датчики, обновляет состояние, выдаёт команду в связанных осях (вперёд, влево, курс, высота) и смешивает её в четыре скорости винтов. Стабилизация — ПИД по высоте, углам и угловым скоростям. Нос доворачивается по направлению участка. После `LANDED` цикл жив, моторы выключены.
