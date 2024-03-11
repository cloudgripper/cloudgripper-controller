#ifndef ROBOT_COMMAND_H
#define ROBOT_COMMAND_H

#include <Arduino.h>

struct RobotCommand {
  enum CommandType {
    MOVE_TO,
    STEP,
    ROTATE_GRIPPER,
    OPEN_CLOSE_GRIPPER,
    UP_DOWN_GRIPPER,
    CALIBRATE,
    STEP_DISTANCE_INCREMENT,
    STEP_DISTANCE_DECREMENT,
    GET_STATE,
    NONE // Default, no command
  } type = NONE;

  float x = 0.0f;
  float y = 0.0f;
  String direction = "";
  int angle = 0;
  int stepDistance = 5; // 5 mm
};

#endif // ROBOT_COMMAND_H