#include "robot_config.h"

// Plotter definitions
char BUFFER[STRING_SIZE + 1];

// Gcode definitions
unsigned int INDEX = 0; // buffer index
int START, FINISH;
float X, Y; // gcode float values held here

// Flags
bool CW = true;   // Does not affect motor direction
bool CCW = false; // Does not affect motor direction

// Variables
bool DIRECTION1;
bool DIRECTION2;
