/*
 * mavic_joystick.c — ручное управление DJI Mavic 2 Pro с пульта RadioMaster TX12.
 *
 * Mode 2:
 *   левый стик вверх/вниз  — скорость набора высоты, центр стика держит высоту;
 *   левый стик влево/вправо — скорость рыскания;
 *   правый стик вверх/вниз — тангаж (от себя — вперёд);
 *   правый стик влево/вправо — крен (вправо — вправо).
 *
 * Внутренний контур и смешивание четырёх винтов взяты из штатного
 * контроллера mavic2pro.c дистрибутива Webots.
 * Отдельного акселерометра у модели нет: крен/тангаж/рыскание читаются
 * с inertial unit, угловые скорости — с gyro, высота — с GPS (ось Z, ENU).
 *
 * В заголовке Webots чтение оси называется wb_joystick_get_axis_value().
 * Ниже оно доступно и под именем wb_joystick_get_axis(), как в задании.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/joystick.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define wb_joystick_get_axis(axis) wb_joystick_get_axis_value(axis)

#define CLAMP(value, low, high) \
  ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

/* Диапазон wb_joystick_get_axis_value(): от -32768 до 32767. */
#define AXIS_FULL_SCALE 32767.0

/*
 * Индексы осей USB-джойстика EdgeTX в порядке каналов AETR (Mode 2):
 *   0 Aileron  — правый стик, горизонталь, крен;
 *   1 Elevator — правый стик, вертикаль, тангаж;
 *   2 Throttle — левый стик, вертикаль, газ;
 *   3 Rudder   — левый стик, горизонталь, рыскание.
 * Если пульт отдаёт другой порядок, меняются только эти четыре числа.
 */
#define AXIS_ROLL 0
#define AXIS_PITCH 1
#define AXIS_THROTTLE 2
#define AXIS_YAW 3

/*
 * Знак сырого значения. +1 оставляет направление ОС, -1 переворачивает.
 * На TX12 в режиме USB Joystick стик «вверх» часто приходит как отрицательная
 * ось. Отрицательный знак переводит «вверх/от себя» в команду +1.
 * Если дрон летит в обратную сторону — смените знак нужной оси на противоположный.
 */
#define SIGN_ROLL 1.0
#define SIGN_PITCH -1.0
#define SIGN_THROTTLE -1.0
#define SIGN_YAW 1.0

#define DEADZONE 0.08 /* доля полного хода; внутри зоны стик считается нулём */

/* Добавки к стабилизации при полном отклонении стика — те же величины,
   что у клавиатуры в mavic2pro.c: вперёд 2.0, стрейф 1.0, рыскание 1.3. */
#define K_PITCH_STICK 2.0
#define K_ROLL_STICK 1.0
#define K_YAW_STICK 1.3
#define K_YAW_D 0.45 /* гасит вращение, когда стик рыскания отпущен */

#define ALTITUDE_RATE 1.2 /* м/с набора или снижения при газе на упоре */
#define ALTITUDE_MIN 0.0
#define ALTITUDE_MAX 40.0

/* Константы висения из mavic2pro.c. Менять их имеет смысл только вместе
   с моделью винтов. */
#define K_VERTICAL_THRUST 68.5
#define K_VERTICAL_OFFSET 0.6
#define K_VERTICAL_P 3.0
#define K_ROLL_P 50.0
#define K_PITCH_P 30.0

#define MOTOR_LIMIT 500.0 /* рад/с, запас до maxVelocity винтов (576) */
#define STATUS_PERIOD 1.0 /* с; 0 отключает периодический вывод осей */

/* Сырое значение оси -> команда -1..1 с мёртвой зоной у нуля.
   За зоной шкала снова растягивается на весь оставшийся ход,
   чтобы сразу за границей команда начиналась с нуля, а не с прыжка. */
static double normalize_axis(int raw, double sign) {
  double value = sign * ((double)raw / AXIS_FULL_SCALE);
  value = CLAMP(value, -1.0, 1.0);

  if (fabs(value) <= DEADZONE)
    return 0.0;

  {
    double span = 1.0 - DEADZONE;
    double magnitude = (fabs(value) - DEADZONE) / span;
    return (value > 0.0) ? magnitude : -magnitude;
  }
}

/* Кнопки пульта. Webots отдаёт их по одной, как клавиши клавиатуры:
   каждый вызов wb_joystick_get_pressed_button() возвращает следующую
   зажатую кнопку или -1, когда список закончился. */
static bool button_is_pressed(int wanted) {
  bool found = false;
  int button = wb_joystick_get_pressed_button();
  while (button >= 0) {
    if (button == wanted)
      found = true;
    button = wb_joystick_get_pressed_button();
  }
  return found;
}

int main(int argc, char **argv) {
  WbDeviceTag imu;
  WbDeviceTag gps;
  WbDeviceTag gyro;
  WbDeviceTag camera_roll_motor;
  WbDeviceTag camera_pitch_motor;
  WbDeviceTag front_left_led;
  WbDeviceTag front_right_led;
  WbDeviceTag front_left_motor;
  WbDeviceTag front_right_motor;
  WbDeviceTag rear_left_motor;
  WbDeviceTag rear_right_motor;
  WbDeviceTag motors[4];
  int timestep;
  int motor_index;

  double target_altitude = 0.0;
  bool altitude_latched = false;
  bool motors_enabled = true;
  bool button0_was_down = false;
  bool joystick_was_connected = false;
  bool origin_set = false;
  double next_status_time = 0.0;

  (void)argc;
  (void)argv;

  wb_robot_init();
  timestep = (int)wb_robot_get_basic_time_step();
  if (timestep <= 0)
    timestep = 8;

  /* Датчики штатной модели Mavic 2 Pro. */
  imu = wb_robot_get_device("inertial unit");
  gps = wb_robot_get_device("gps");
  gyro = wb_robot_get_device("gyro");
  front_left_led = wb_robot_get_device("front left led");
  front_right_led = wb_robot_get_device("front right led");
  camera_roll_motor = wb_robot_get_device("camera roll");
  camera_pitch_motor = wb_robot_get_device("camera pitch");

  if (imu == 0 || gps == 0 || gyro == 0) {
    fprintf(stderr, "Нет inertial unit, gps или gyro. Контроллер рассчитан на узел Mavic2Pro.\n");
    wb_robot_cleanup();
    return EXIT_FAILURE;
  }

  wb_inertial_unit_enable(imu, timestep);
  wb_gps_enable(gps, timestep);
  wb_gyro_enable(gyro, timestep);

  /* Четыре винта в режиме скорости: положение «бесконечность», команда — рад/с.
     Правые и задние винты в модели крутятся в обратную сторону, поэтому
     при записи скорости их команда меняет знак. */
  front_left_motor = wb_robot_get_device("front left propeller");
  front_right_motor = wb_robot_get_device("front right propeller");
  rear_left_motor = wb_robot_get_device("rear left propeller");
  rear_right_motor = wb_robot_get_device("rear right propeller");
  motors[0] = front_left_motor;
  motors[1] = front_right_motor;
  motors[2] = rear_left_motor;
  motors[3] = rear_right_motor;

  for (motor_index = 0; motor_index < 4; ++motor_index) {
    if (motors[motor_index] == 0) {
      fprintf(stderr, "Не найден один из четырёх propeller. Проверьте имена устройств.\n");
      wb_robot_cleanup();
      return EXIT_FAILURE;
    }
    wb_motor_set_position(motors[motor_index], INFINITY);
    wb_motor_set_velocity(motors[motor_index], 1.0);
  }

  /* Пульт компьютера, не устройство робота. Webots берёт первый подключённый джойстик. */
  wb_joystick_enable(timestep);

  printf("Mavic 2 Pro, Mode 2, пульт USB.\n");
  printf("Левый стик: газ (центр — удержание высоты) и рыскание.\n");
  printf("Правый стик: тангаж и крен.\n");
  printf("Кнопка 0 переключает моторы. Оси AETR: %d %d %d %d.\n",
         AXIS_ROLL, AXIS_PITCH, AXIS_THROTTLE, AXIS_YAW);

  while (wb_robot_step(timestep) != -1) {
    const double dt = timestep / 1000.0;
    const double time = wb_robot_get_time();

    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    const double roll = rpy[0];
    const double pitch = rpy[1];
    const double *gyro_values = wb_gyro_get_values(gyro);
    const double roll_rate = gyro_values[0];
    const double pitch_rate = gyro_values[1];
    const double yaw_rate = gyro_values[2];
    const double altitude = wb_gps_get_values(gps)[2];

    bool connected = wb_joystick_is_connected();
    double roll_stick = 0.0;
    double pitch_stick = 0.0;
    double throttle_stick = 0.0;
    double yaw_stick = 0.0;
    int raw_roll = 0;
    int raw_pitch = 0;
    int raw_throttle = 0;
    int raw_yaw = 0;

    /* Камера смотрит в горизонт и компенсирует колебания корпуса, как в примере. */
    if (camera_roll_motor)
      wb_motor_set_position(camera_roll_motor, -0.115 * roll_rate);
    if (camera_pitch_motor)
      wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_rate);

    if (connected && !joystick_was_connected) {
      const char *model = wb_joystick_get_model();
      printf("Пульт подключён: %s, осей %d, кнопок читается через get_pressed_button().\n",
             (model != NULL) ? model : "неизвестная модель",
             wb_joystick_get_number_of_axes());
      if (wb_joystick_get_number_of_axes() < 4)
        fprintf(stderr, "У джойстика меньше четырёх осей. Проверьте режим USB Joystick на TX12.\n");
    }
    if (!connected && joystick_was_connected)
      printf("Пульт отключён. Стики обнулены, высота удерживается.\n");
    joystick_was_connected = connected;

    if (connected && wb_joystick_get_number_of_axes() >= 4) {
      raw_roll = wb_joystick_get_axis(AXIS_ROLL);
      raw_pitch = wb_joystick_get_axis(AXIS_PITCH);
      raw_throttle = wb_joystick_get_axis(AXIS_THROTTLE);
      raw_yaw = wb_joystick_get_axis(AXIS_YAW);

      roll_stick = normalize_axis(raw_roll, SIGN_ROLL);
      pitch_stick = normalize_axis(raw_pitch, SIGN_PITCH);
      throttle_stick = normalize_axis(raw_throttle, SIGN_THROTTLE);
      yaw_stick = normalize_axis(raw_yaw, SIGN_YAW);

      /* Кнопка 0 — предохранитель моторов. На TX12 это часто моментальный тумблер SH. */
      {
        bool button0_down = button_is_pressed(0);
        if (button0_down && !button0_was_down) {
          motors_enabled = !motors_enabled;
          printf("%s\n", motors_enabled ? "Моторы включены." : "Моторы выключены.");
        }
        button0_was_down = button0_down;
      }
    } else {
      button0_was_down = false;
    }

    /* Первое достоверное GPS фиксирует высоту, которую дрон держит,
       пока газ в мёртвой зоне. Иначе кубический контур сразу потянет вверх. */
    if (!origin_set && isfinite(altitude)) {
      target_altitude = altitude - K_VERTICAL_OFFSET;
      altitude_latched = true;
      origin_set = true;
    }

    /*
     * Газ — это скорость изменения заданной высоты, а не сама тяга.
     * В мёртвой зоне цель защёлкивается один раз: если обновлять её каждый шаг,
     * ошибка высоты всегда нулевая и дрон не возвращается на отметку.
     * Равновесие кубического закона: altitude = target_altitude + K_VERTICAL_OFFSET,
     * поэтому в цель записывается текущая высота без этого смещения.
     */
    if (origin_set && motors_enabled) {
      if (throttle_stick == 0.0) {
        if (!altitude_latched) {
          target_altitude = altitude - K_VERTICAL_OFFSET;
          altitude_latched = true;
          printf("Удержание высоты %.2f м\n", altitude);
        }
      } else {
        altitude_latched = false;
        target_altitude += throttle_stick * ALTITUDE_RATE * dt;
        target_altitude = CLAMP(target_altitude, ALTITUDE_MIN, ALTITUDE_MAX);
      }
    }

    if (motors_enabled && origin_set) {
      /* Стик вправо даёт отрицательную добавку крена: в этом смешивании
         так опускается правый борт и дрон летит вправо.
         Стик тангажа «от себя» (+1) даёт отрицательную добавку и опускает нос. */
      const double roll_disturbance = CLAMP(-roll_stick * K_ROLL_STICK, -K_ROLL_STICK, K_ROLL_STICK);
      const double pitch_disturbance = CLAMP(-pitch_stick * K_PITCH_STICK, -K_PITCH_STICK, K_PITCH_STICK);
      /* Стик рыскания вправо — отрицательная команда, как клавиша Right в примере.
         Вычитание скорости с гироскопа останавливает вращение в центре стика. */
      const double yaw_disturbance = CLAMP(-yaw_stick * K_YAW_STICK, -K_YAW_STICK, K_YAW_STICK);

      const double roll_input = K_ROLL_P * CLAMP(roll, -1.0, 1.0) + roll_rate + roll_disturbance;
      const double pitch_input = K_PITCH_P * CLAMP(pitch, -1.0, 1.0) + pitch_rate + pitch_disturbance;
      const double yaw_input = yaw_disturbance - K_YAW_D * yaw_rate;

      const double altitude_error =
        CLAMP(target_altitude - altitude + K_VERTICAL_OFFSET, -1.0, 1.0);
      const double vertical_input = K_VERTICAL_P * altitude_error * altitude_error * altitude_error;

      double front_left = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input;
      double front_right = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input;
      double rear_left = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input;
      double rear_right = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input;

      front_left = CLAMP(front_left, -MOTOR_LIMIT, MOTOR_LIMIT);
      front_right = CLAMP(front_right, -MOTOR_LIMIT, MOTOR_LIMIT);
      rear_left = CLAMP(rear_left, -MOTOR_LIMIT, MOTOR_LIMIT);
      rear_right = CLAMP(rear_right, -MOTOR_LIMIT, MOTOR_LIMIT);

      wb_motor_set_velocity(front_left_motor, front_left);
      wb_motor_set_velocity(front_right_motor, -front_right);
      wb_motor_set_velocity(rear_left_motor, -rear_left);
      wb_motor_set_velocity(rear_right_motor, rear_right);

      if (front_left_led)
        wb_led_set(front_left_led, 1);
      if (front_right_led)
        wb_led_set(front_right_led, 1);
    } else {
      /* Выключенные моторы или ещё нет GPS — винты почти стоят, как в паузе примера. */
      for (motor_index = 0; motor_index < 4; ++motor_index)
        wb_motor_set_velocity(motors[motor_index], 1.0);
      if (front_left_led)
        wb_led_set(front_left_led, 0);
      if (front_right_led)
        wb_led_set(front_right_led, 0);
    }

    if (STATUS_PERIOD > 0.0 && time >= next_status_time && connected) {
      printf("raw %6d %6d %6d %6d | stick r=%+.2f p=%+.2f thr=%+.2f y=%+.2f | z=%.2f цель=%.2f\n",
             raw_roll, raw_pitch, raw_throttle, raw_yaw,
             roll_stick, pitch_stick, throttle_stick, yaw_stick,
             altitude, target_altitude + K_VERTICAL_OFFSET);
      next_status_time = time + STATUS_PERIOD;
    }
  }

  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
