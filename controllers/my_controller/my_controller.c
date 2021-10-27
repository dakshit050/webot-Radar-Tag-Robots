#include <stdlib.h>
#include <string.h>
#include <webots/motor.h>
#include <webots/range_finder.h>
#include <webots/robot.h>
#include <webots/utils/system.h>

#define SPEED 4
#define TIME_STEP 32
//Define the speed of the robot and the simulation time.

int main() 
{
    WbDeviceTag range_finder, left_motor, right_motor;
  int range_finder_width, range_finder_height;
  int left_speed, right_speed, i;
  float distance;
  
  wb_robot_init();
  
    range_finder = wb_robot_get_device("range finder");
  wb_range_finder_enable(range_finder, TIME_STEP);
  range_finder_width = wb_range_finder_get_width(range_finder);
  range_finder_height = wb_range_finder_get_height(range_finder);
  
    /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("leftmotor");
  right_motor = wb_robot_get_device("rightmotor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
    while (wb_robot_step(TIME_STEP) != -1) 
  {
  
  }
}