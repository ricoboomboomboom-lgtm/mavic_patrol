/*
 * mavic_patrol.c — контроллер DJI Mavic 2 Pro для Webots.
 *
 * После старта дрон взлетает на FLIGHT_ALTITUDE и зависает.
 * Затем смещается на ближний левый угол прямоугольника, который лежит
 * прямо перед носом (по курсу на момент старта), и обходит его периметр
 * по часовой стрелке, пока симуляцию не остановят.
 *
 * Смешивание моторов и внутренний контур крена/тангажа повторяют
 * штатный контроллер mavic2pro.c из дистрибутива Webots.
 * Координаты мира — ENU (Webots R2022+): X на восток, Y на север, Z вверх.
 * При нулевом рыскании нос смотрит в +X, левый борт — в +Y.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------- параметры задания ---------- */
#define FLIGHT_ALTITUDE 3.0 /* высота полёта, м (над точкой старта) */
#define AREA_LENGTH 8.0     /* длина зоны патрулирования, м (вперёд от дрона) */
#define AREA_WIDTH 6.0      /* ширина зоны патрулирования, м */
#define PATROL_SPEED 1.0    /* линейная скорость облёта, м/с */

/* ---------- модель винта, как в mavic2pro.c ---------- */
#define K_VERTICAL_THRUST 68.5 /* рад/с, при которых модель уже поднимается */
#define K_VERTICAL_OFFSET 0.6  /* эмпирический сдвиг нуля вертикального контура */
#define K_VERTICAL_P 2.2       /* П по ошибке высоты, рад/с на метр */
#define K_VERTICAL_D 4.0       /* Д по вертикальной скорости */

#define K_ROLL_P 50.0 /* П крена из примера: рад/с команды на радиан крена */
#define K_PITCH_P 30.0
#define K_ROLL_D 1.0 /* Д крена: коэффициент при скорости с гироскопа */
#define K_PITCH_D 1.0

/* Внешний контур: ошибка положения -> ограниченная скорость -> крен/тангаж.
   Отрицательный тангаж в этом смешивании опускает нос и даёт ход вперёд.
   Положительный крен опускает левый борт и даёт ход влево. */
#define K_POS_P 0.85          /* 1/с, из метров ошибки в м/с задания */
#define K_PITCH_VEL 1.35      /* рад/с команды на (м/с) ошибки продольной скорости */
#define K_ROLL_VEL 0.75       /* то же поперёк; крен в примере жёстче тангажа */
#define PITCH_LIMIT 1.6       /* предел добавки тангажа, как у клавиши «вперёд» */
#define ROLL_LIMIT 1.0        /* предел добавки крена */
#define HOLD_SPEED 0.45       /* м/с, пока дрон доворачивает рыскание на углу */

#define K_YAW_P 1.0           /* рад/с команды на радиан ошибки курса */
#define K_YAW_D 0.25          /* демпфирование скорости рыскания */
#define YAW_LIMIT 0.9         /* предел канала рыскания, без рывка моторов */
#define YAW_SLEW_RATE 0.55    /* рад/с, плавный доворот на 90° (~2.9 с) */
#define YAW_ALIGN 0.12        /* рад, курс считается выставленным */

#define WP_RADIUS 0.40        /* м, допуск прихода в угол */
#define ALT_TOL 0.30          /* м, допуск стабилизации высоты */
#define VZ_TOL 0.18           /* м/с */
#define STABLE_TIME 1.0       /* с непрерывного удержания перед переходом */
#define TURN_TIMEOUT 5.0      /* с, чтобы не застрять на углу */
#define MOTOR_LIMIT 500.0     /* рад/с, ниже maxVelocity винтов (576) */

#define CLAMP(value, low, high) \
  ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

typedef enum {
  IDLE = 0,
  TAKEOFF,
  MOVE_TO_ZONE,
  PATROL
} FlightState;

/* Подфаза только внутри PATROL: прямая сторона, затем плавный доворот. */
typedef enum {
  PATROL_FLY = 0,
  PATROL_TURN
} PatrolPhase;

static const char *state_name(FlightState state) {
  switch (state) {
    case IDLE:
      return "IDLE";
    case TAKEOFF:
      return "TAKEOFF";
    case MOVE_TO_ZONE:
      return "MOVE_TO_ZONE";
    case PATROL:
      return "PATROL";
    default:
      return "?";
  }
}

/* Разница углов в диапазон (-pi, pi]. */
static double wrap_pi(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle <= -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

/* Курс стороны от угла leg к следующему углу, рад в СК мира. */
static double leg_heading(const double corners[4][2], int leg) {
  const int next = (leg + 1) % 4;
  return atan2(corners[next][1] - corners[leg][1], corners[next][0] - corners[leg][0]);
}

/*
 * Четыре угла зоны в плоскости XY.
 * Ближняя кромка проходит через точку старта, длина откладывается вперёд
 * по начальному курсу, ширина симметрична влево и вправо.
 * Обход: 0 ближний левый -> 1 дальний левый -> 2 дальний правый -> 3 ближний правый.
 */
static void build_corners(double x0, double y0, double yaw0, double corners[4][2]) {
  const double forward_x = cos(yaw0);
  const double forward_y = sin(yaw0);
  const double left_x = -sin(yaw0);
  const double left_y = cos(yaw0);
  const double half_w = AREA_WIDTH * 0.5;
  const double left[2] = {left_x * half_w, left_y * half_w};
  const double ahead[2] = {forward_x * AREA_LENGTH, forward_y * AREA_LENGTH};

  corners[0][0] = x0 + left[0];
  corners[0][1] = y0 + left[1];
  corners[1][0] = x0 + ahead[0] + left[0];
  corners[1][1] = y0 + ahead[1] + left[1];
  corners[2][0] = x0 + ahead[0] - left[0];
  corners[2][1] = y0 + ahead[1] - left[1];
  corners[3][0] = x0 - left[0];
  corners[3][1] = y0 - left[1];
}

/* Ошибка точки в связанной СК: вперёд (+X корпуса) и влево (+Y корпуса). */
static void world_to_body(double yaw, double dx, double dy, double *forward, double *left) {
  const double c = cos(yaw);
  const double s = sin(yaw);
  *forward = c * dx + s * dy;
  *left = -s * dx + c * dy;
}

static int device_ok(WbDeviceTag tag, const char *name) {
  if (tag != 0)
    return 1;
  fprintf(stderr, "mavic_patrol: устройство \"%s\" не найдено.\n", name);
  return 0;
}

int main(void) {
  wb_robot_init();

  /* Шаг берётся из WorldInfo.basicTimeStep, для этой модели устойчив шаг 8 мс. */
  const int TIME_STEP = (int)wb_robot_get_basic_time_step();
  if (TIME_STEP <= 0) {
    fprintf(stderr, "mavic_patrol: некорректный basicTimeStep.\n");
    wb_robot_cleanup();
    return EXIT_FAILURE;
  }
  const double dt = TIME_STEP / 1000.0;

  const WbDeviceTag imu = wb_robot_get_device("inertial unit");
  const WbDeviceTag gps = wb_robot_get_device("gps");
  const WbDeviceTag gyro = wb_robot_get_device("gyro");
  const WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  const WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  const WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  const WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");

  if (!device_ok(imu, "inertial unit") || !device_ok(gps, "gps") || !device_ok(gyro, "gyro") ||
      !device_ok(front_left_motor, "front left propeller") ||
      !device_ok(front_right_motor, "front right propeller") ||
      !device_ok(rear_left_motor, "rear left propeller") ||
      !device_ok(rear_right_motor, "rear right propeller")) {
    wb_robot_cleanup();
    return EXIT_FAILURE;
  }

  wb_inertial_unit_enable(imu, TIME_STEP);
  wb_gps_enable(gps, TIME_STEP);
  wb_gyro_enable(gyro, TIME_STEP);

  /* VELOCITY: бесконечная цель по положению, команда — скорость вращения. */
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  for (int i = 0; i < 4; ++i) {
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], 1.0);
  }

  FlightState state = IDLE;
  PatrolPhase patrol_phase = PATROL_FLY;
  int leg = 0;
  double turn_time = 0.0;
  double stable_time = 0.0;
  double yaw_slew = 0.0;

  int origin_valid = 0;
  double home_x = 0.0, home_y = 0.0, home_z = 0.0, yaw0 = 0.0;
  double corners[4][2] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};

  int deriv_ready = 0;
  double prev_x = 0.0, prev_y = 0.0, prev_z = 0.0;
  double vx_f = 0.0, vy_f = 0.0, vz_f = 0.0;

  printf("mavic_patrol: высота %.1f м, зона %.1f x %.1f м, скорость %.1f м/с\n", FLIGHT_ALTITUDE, AREA_LENGTH,
         AREA_WIDTH, PATROL_SPEED);
  fflush(stdout);

  while (wb_robot_step(TIME_STEP) != -1) {
    /* Указатели датчиков живут только до следующего вызова того же устройства. */
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    const double roll = rpy[0];
    const double pitch = rpy[1];
    const double yaw = rpy[2];

    const double *gps_values = wb_gps_get_values(gps);
    const double x = gps_values[0];
    const double y = gps_values[1];
    const double z = gps_values[2];

    const double *gyro_values = wb_gyro_get_values(gyro);
    const double roll_rate = gyro_values[0];
    const double pitch_rate = gyro_values[1];
    const double yaw_rate = gyro_values[2];

    if (!isfinite(x) || !isfinite(y) || !isfinite(z) || !isfinite(yaw))
      continue;

    double vx = 0.0, vy = 0.0, vz = 0.0;
    if (deriv_ready) {
      /* Скорость по приращению GPS. Лёгкий фильтр сглаживает ступеньку датчика. */
      const double vx_raw = (x - prev_x) / dt;
      const double vy_raw = (y - prev_y) / dt;
      const double vz_raw = (z - prev_z) / dt;
      const double alpha = 0.35;
      vx_f += alpha * (vx_raw - vx_f);
      vy_f += alpha * (vy_raw - vy_f);
      vz_f += alpha * (vz_raw - vz_f);
      vx = vx_f;
      vy = vy_f;
      vz = vz_f;
    }
    prev_x = x;
    prev_y = y;
    prev_z = z;
    deriv_ready = 1;

    /* IDLE: один достоверный замер фиксирует старт и строит прямоугольник. */
    if (!origin_valid) {
      home_x = x;
      home_y = y;
      home_z = z;
      yaw0 = yaw;
      yaw_slew = yaw;
      build_corners(home_x, home_y, yaw0, corners);
      origin_valid = 1;
      state = TAKEOFF;
      printf("[IDLE] старт x=%.2f y=%.2f z=%.2f курс=%.1f град\n", home_x, home_y, home_z, yaw0 * 180.0 / M_PI);
      printf("[TAKEOFF] набор %.2f м\n", home_z + FLIGHT_ALTITUDE);
      for (int i = 0; i < 4; ++i)
        printf("  угол %d: x=%.2f y=%.2f\n", i, corners[i][0], corners[i][1]);
      fflush(stdout);
    }

    const double flight_z = home_z + FLIGHT_ALTITUDE;
    /*
     * Вертикальный ПД.
     * В примере mavic2pro ноль тяги смещён на K_VERTICAL_OFFSET: при нулевой
     * ошибке аппарат садится выше числа, записанного в цель. Здесь цель
     * заранее занижена на ту же величину, поэтому равновесие П-члена
     * совпадает с FLIGHT_ALTITUDE над точкой старта.
     * Д-член гасит вертикальную скорость и заменяет кубическую мягкость примера.
     */
    const double altitude_goal = flight_z - K_VERTICAL_OFFSET;
    const double alt_err = CLAMP(altitude_goal - z + K_VERTICAL_OFFSET, -1.0, 1.0);
    const double vertical_input = K_VERTICAL_P * alt_err + K_VERTICAL_D * (-vz);

    double goal_x = home_x;
    double goal_y = home_y;
    double yaw_goal = yaw0;
    double speed_cap = HOLD_SPEED;

    switch (state) {
      case IDLE:
        break;

      case TAKEOFF:
        /* Удерживаем точку старта и исходный курс, пока набирается высота. */
        goal_x = home_x;
        goal_y = home_y;
        yaw_goal = yaw0;
        speed_cap = HOLD_SPEED;
        if (fabs(flight_z - z) < ALT_TOL && fabs(vz) < VZ_TOL)
          stable_time += dt;
        else
          stable_time = 0.0;
        if (stable_time >= STABLE_TIME) {
          state = MOVE_TO_ZONE;
          stable_time = 0.0;
          printf("[MOVE_TO_ZONE] вход в зону, угол 0 (%.2f, %.2f), высота %.2f м\n", corners[0][0], corners[0][1], z);
          fflush(stdout);
        }
        break;

      case MOVE_TO_ZONE:
        /*
         * Зона начинается у дрона, первая точка периметра — ближний левый угол.
         * Курс не меняем: боковое смещение даёт канал крена, без лишнего разворота.
         */
        goal_x = corners[0][0];
        goal_y = corners[0][1];
        yaw_goal = yaw0;
        speed_cap = PATROL_SPEED;
        {
          const double dist = hypot(goal_x - x, goal_y - y);
          const double speed = hypot(vx, vy);
          if (dist < WP_RADIUS && speed < 0.35) {
            state = PATROL;
            patrol_phase = PATROL_FLY;
            leg = 0;
            printf("[PATROL] обход периметра, сторона 0\n");
            fflush(stdout);
          }
        }
        break;

      case PATROL:
        if (patrol_phase == PATROL_FLY) {
          const int next = (leg + 1) % 4;
          goal_x = corners[next][0];
          goal_y = corners[next][1];
          yaw_goal = leg_heading(corners, leg);
          speed_cap = PATROL_SPEED;
          if (hypot(goal_x - x, goal_y - y) < WP_RADIUS) {
            /* Угол достигнут: зависаем и плавно меняем курс на следующую сторону. */
            patrol_phase = PATROL_TURN;
            turn_time = 0.0;
            printf("[PATROL] угол %d, доворот курса\n", next);
            fflush(stdout);
          }
        } else {
          const int next = (leg + 1) % 4;
          goal_x = corners[next][0];
          goal_y = corners[next][1];
          yaw_goal = leg_heading(corners, next);
          speed_cap = HOLD_SPEED;
          turn_time += dt;
          const int yaw_ok = fabs(wrap_pi(yaw_goal - yaw)) < YAW_ALIGN && fabs(yaw_rate) < 0.30;
          const int on_corner = hypot(goal_x - x, goal_y - y) < WP_RADIUS * 2.0;
          if (yaw_ok && (on_corner || turn_time > TURN_TIMEOUT)) {
            leg = next;
            patrol_phase = PATROL_FLY;
            printf("[PATROL] сторона %d\n", leg);
            fflush(stdout);
          }
        }
        break;
    }

    /*
     * Задание курса идёт через ограничитель скорости, а не скачком.
     * На прямых yaw_goal почти равен текущему курсу, ограничение не мешает.
     * На углу задание сдвигается на 90° за несколько секунд.
     */
    double yaw_step = wrap_pi(yaw_goal - yaw_slew);
    const double max_yaw_step = YAW_SLEW_RATE * dt;
    yaw_step = CLAMP(yaw_step, -max_yaw_step, max_yaw_step);
    yaw_slew = wrap_pi(yaw_slew + yaw_step);

    double err_forward = 0.0, err_left = 0.0;
    world_to_body(yaw, goal_x - x, goal_y - y, &err_forward, &err_left);
    double v_forward = 0.0, v_left = 0.0;
    world_to_body(yaw, vx, vy, &v_forward, &v_left);

    /*
     * Горизонтальный ПД.
     * П по положению даёт желаемую скорость и сразу ограничивается speed_cap,
     * поэтому на длинной стороне скорость не растёт вместе с оставшимся путём.
     * Рассогласование скорости — Д-составляющая положения: она наклоняет дрон,
     * пока фактическая скорость не сравняется с заданной, и тормозит у угла.
     */
    const double v_forward_des = CLAMP(K_POS_P * err_forward, -speed_cap, speed_cap);
    const double v_left_des = CLAMP(K_POS_P * err_left, -speed_cap, speed_cap);
    const double pitch_disturbance =
        CLAMP(-K_PITCH_VEL * (v_forward_des - v_forward), -PITCH_LIMIT, PITCH_LIMIT);
    const double roll_disturbance = CLAMP(K_ROLL_VEL * (v_left_des - v_left), -ROLL_LIMIT, ROLL_LIMIT);

    /*
     * Рыскание: П по разнице сглаженного задания и курса с ИНС,
     * Д — скорость рыскания с гироскопа, со знаком против вращения.
     */
    const double yaw_err = wrap_pi(yaw_slew - yaw);
    const double yaw_disturbance = CLAMP(K_YAW_P * yaw_err - K_YAW_D * yaw_rate, -YAW_LIMIT, YAW_LIMIT);

    /*
     * Внутренний контур, формула из mavic2pro.c.
     * П тянет крен и тангаж к нулю, Д (скорость с гироскопа) гасит колебания.
     * Добавки внешнего контура смещают равновесие: аппарат замирает с малым
     * наклоном, и горизонтальная тяга винтов везёт его в нужную сторону.
     */
    const double roll_input = K_ROLL_P * CLAMP(roll, -1.0, 1.0) + K_ROLL_D * roll_rate + roll_disturbance;
    const double pitch_input = K_PITCH_P * CLAMP(pitch, -1.0, 1.0) + K_PITCH_D * pitch_rate + pitch_disturbance;
    const double yaw_input = yaw_disturbance;

    double front_left = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input;
    double front_right = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input;
    double rear_left = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input;
    double rear_right = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input;

    front_left = CLAMP(front_left, -MOTOR_LIMIT, MOTOR_LIMIT);
    front_right = CLAMP(front_right, -MOTOR_LIMIT, MOTOR_LIMIT);
    rear_left = CLAMP(rear_left, -MOTOR_LIMIT, MOTOR_LIMIT);
    rear_right = CLAMP(rear_right, -MOTOR_LIMIT, MOTOR_LIMIT);

    /* Правые и задний левый винты в модели крутятся в обратную сторону. */
    wb_motor_set_velocity(front_left_motor, front_left);
    wb_motor_set_velocity(front_right_motor, -front_right);
    wb_motor_set_velocity(rear_left_motor, -rear_left);
    wb_motor_set_velocity(rear_right_motor, rear_right);
  }

  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
