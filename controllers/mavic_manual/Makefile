# Webots R2025a controller makefile. Builds with the compiler shipped with Webots.
# From a shell where WEBOTS_HOME points at the Webots install:
#   make
#   make clean

C_SOURCES = mavic_manual.c flight_types.c common.c config.c pid.c input_manager.c sensor_manager.c state_estimator.c attitude_controller.c altitude_controller.c position_controller.c motor_mixer.c safety_manager.c telemetry.c

CFLAGS = -std=c11 -Wall -Wextra

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
