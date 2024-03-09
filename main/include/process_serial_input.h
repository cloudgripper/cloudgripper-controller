// Checks if PROCESS_SERIAL_INPUT_H macro is defined
#ifndef PROCESS_SERIAL_INPUT_H
// Define PROCESS_SERIAL_INPUT_H macro
#define PROCESS_SERIAL_INPUT_H

#include <Arduino.h>
#include "robot_config.h"
#include "cloudgripper.h"
#include "robot_command.h"

void menu()
{
    Serial.println(F(""));
    Serial.println(F("  ------------------------------------------------------"));
    Serial.println(F("                         MENU                           "));
    Serial.println(F("  ------------------------------------------------------"));
    Serial.println(F("    G00 X## Y## -------- goto XY (## mm)"));
    Serial.println(F("    T1 ----------------- calibrate Steppers"));
    Serial.println(F("    DD ----------------- Step Right"));
    Serial.println(F("    LL ----------------- Step Left"));
    Serial.println(F("    FF ----------------- Step Forward"));
    Serial.println(F("    BB ----------------- Step Backward"));
    Serial.println(F("    R# ----------------- Grip Rotate (# degree)"));
    Serial.println(F("    O# ----------------- Grip Open/Close (# degree)"));
    Serial.println(F("    P# ----------------- Grip Up/Down (# degree)"));
    Serial.println(F("    R(angle) ----------- Rotate with angle E.g. R90"));
    Serial.println(F("  ======================================================"));
}

// Parse Serial Input
RobotCommand parseRobotCommand(String string)
{
    String inputString = string;
    String subString;
    RobotCommand command;

    // Convert string to upper case
    inputString.toUpperCase();

    // G00 - linear move
    if (inputString.startsWith("G00"))
    {
        command.type = RobotCommand::MOVE_TO;
        // Extract X
        START = inputString.indexOf('X');
        if (!(START < 0))
        {
            FINISH = START + 8;
            subString = inputString.substring(START + 1, FINISH + 1);
            command.x = subString.toFloat();
        }

        // Extract Y
        START = inputString.indexOf('Y');
        if (!(START < 0))
        {
            FINISH = START + 8;
            subString = inputString.substring(START + 1, FINISH + 1);
            command.y = subString.toFloat();
        }
        // Constraint movement
        if (command.x > 160)
        {
            command.x = 160;
        }
        if (command.y > 230)
        {
            command.y = 230;
        }
    }

    // Menu
    else if (inputString.startsWith("MENU"))
    {
        menu();
    }
    // Step in either direction
    else if (inputString.startsWith("DD") || inputString.startsWith("LL") || 
             inputString.startsWith("FF") || inputString.startsWith("BB"))
    {
        command.type = RobotCommand::STEP;
        if (inputString.startsWith("DD")) command.direction = "right";
        else if (inputString.startsWith("LL")) command.direction = "left";
        else if (inputString.startsWith("FF")) command.direction = "forward";
        else command.direction = "backward";
    }
    // Rotate gripper
    else if (inputString.startsWith("R"))
    {
        command.type = RobotCommand::ROTATE_GRIPPER;

        // Extract angle
        START = inputString.indexOf('R');
        if (!(START < 0))
        {
            FINISH = START + 8;
            subString = inputString.substring(START + 1, FINISH + 1);
            command.angle = subString.toInt();
        }
    }
    // Open/Close Gripper
    else if (inputString.startsWith("O"))
    {
        command.type = RobotCommand::OPEN_CLOSE_GRIPPER;

        // Extract angle
        START = inputString.indexOf('O');
        if (!(START < 0))
        {
            FINISH = START + 8;
            subString = inputString.substring(START + 1, FINISH + 1);
            command.angle = subString.toInt();
        }
        if (command.angle > 90)
        {
            command.angle = 90;
        }
        if (command.angle < 0)
        {
            command.angle = 0;
        }
    }
    // Move gripper in Z-axis
    else if (inputString.startsWith("P"))
    {
        command.type = RobotCommand::UP_DOWN_GRIPPER;

        // Extract angle
        START = inputString.indexOf('P');
        if (!(START < 0))
        {
            FINISH = START + 8;
            subString = inputString.substring(START + 1, FINISH + 1);
            command.angle = subString.toInt();
        }
        if (command.angle > 180)
        {
            command.angle = 180;
        }
        if (command.angle < 0)
        {
            command.angle = 0;
        }
    }
    // Calibrate
    else if (inputString.startsWith("T1"))
    {
        command.type = RobotCommand::CALIBRATE;
    }
    // Increase stepDistance by 1 mm
    else if (inputString.startsWith("X"))
    {
        command.stepDistance += 1;
    }
    // Decrease stepDistance by 1 mm
    else if (inputString.startsWith("Z"))
    {
        command.stepDistance -= 1;
    }
    // Detect state request
    else if (inputString.startsWith("S"))
    {
        command.type = RobotCommand::GET_STATE;
    }

    return command;
}

#endif // PROCESS_SERIAL_INPUT_H