/*
 * File:          z513399_MTRN4110_PhaseA.c
 * Date:          4/6/2020
 * Description:   Some controller
 * Author:        Gordon
 * Modifications:
 * Hard-coding: Nope
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/distance_sensor.h>
#include <webots/device.h>
#include <stdio.h>

#define PI 3.14159265358979323846

#define TIME_STEP 64
#define MAX_SPEED 6.28

#define COLLISION_THRESHOLD 980
#define WALL_SENSOR_THRESHOLD 800
#define TURN_GAIN 4

void updateDirection(WbDeviceTag left_motor, WbDeviceTag right_motor, WbDeviceTag IMU, const double IMU_zero, double *IMU_next);


int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  // read path file into commands
  printf("Reading file...\n");
  FILE *fptr;
  fptr = fopen("../../PathPlan.txt","r");
  if(fptr == NULL) {
    printf("Error!\n");   
    return(1);             
  }
  char commands[255] = {0};
  fgets(commands, 255, fptr);
  printf("%s\n", commands);
  
  printf("Initialising...\n");
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag ds_forward = wb_robot_get_device("dsForward");
  WbDeviceTag ds_left = wb_robot_get_device("dsLeft");
  WbDeviceTag ds_right = wb_robot_get_device("dsRight");
  WbDeviceTag IMU = wb_robot_get_device("inertial unit");
  
  // set the target position of the motors
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  wb_motor_set_acceleration(left_motor, INFINITY);
  wb_motor_set_acceleration(right_motor, INFINITY);
  
  wb_distance_sensor_enable(ds_forward, TIME_STEP);
  wb_distance_sensor_enable(ds_left, TIME_STEP);
  wb_distance_sensor_enable(ds_right, TIME_STEP);
  wb_inertial_unit_enable(IMU, TIME_STEP);
  
  char heading[4] = {'N', 'E', 'S', 'W'};
  int grid[2] = {commands[0] - '0', commands[1] - '0'};
  double dsForwardVal = 0;
  double dsLeftVal = 0;
  double dsRightVal = 0;
  
  int headingIndex = 0;
  switch(commands[2]) {
    case 'N': headingIndex = 0; break;
    case 'E': headingIndex = 1; break;
    case 'S': headingIndex = 2; break;
    case 'W': headingIndex = 3; break;
  }
  // Zero the IMU measurement with initial heading
  wb_robot_step(TIME_STEP);
  const double *IMU_val;
  IMU_val = wb_inertial_unit_get_roll_pitch_yaw(IMU);
  const double IMU_zero = *(IMU_val + 2);
  double IMU_current = 0;
  double IMU_next = IMU_current;
  
  // ----------------------------------------------------------------------------------------
  int i = 3;
  int delay = 0;
  printf("Moving...\n");
  // timer based control
  while (wb_robot_step(TIME_STEP) != -1 && commands[i] != '\0') {
    // maybe: dynamic delay depending on velocity
    // Sensor info
    // Distance sensor
    dsForwardVal = wb_distance_sensor_get_value(ds_forward);
    dsLeftVal = wb_distance_sensor_get_value(ds_left);
    dsRightVal = wb_distance_sensor_get_value(ds_right);
    printf("Sensor Values: %f %f %f\n", dsLeftVal, dsForwardVal, dsRightVal);
    // IMU measurements
    //IMU_val = wb_inertial_unit_get_roll_pitch_yaw(IMU);
    //IMU_current = *(IMU_val + 2) - IMU_zero;
    //printf("Yaw: %f ", IMU_current);
    //printf("IMU: Roll: %f Pitch: %f Yaw: %f\n", *IMUVal, *(IMUVal + 1), *(IMUVal + 2));
    
    printf("Current command: %d - %c, ", i, commands[i]);
    printf("Grid: (%d, %d), ", grid[0], grid[1]);
    printf("Heading: %c, ", heading[headingIndex]);
    printf("Left Wall: ");
    if(dsLeftVal > WALL_SENSOR_THRESHOLD) {
      printf("Y, ");
    } else {
      printf("N, ");
    }
    printf("Front Wall: ");
    if(dsForwardVal > WALL_SENSOR_THRESHOLD) {
      printf("Y, ");
    } else {
      printf("N, ");
    }
    printf("Right Wall: ");
    if(dsRightVal > WALL_SENSOR_THRESHOLD) {
      printf("Y\n");
    } else {
      printf("N\n");
    }
    
    
    // Process commands
    if(commands[i] == 'F') {
      switch(heading[headingIndex]) {
        case 'N': grid[0] -= 1; break;
        case 'E': grid[1] += 1; break;
        case 'S': grid[0] += 1; break;
        case 'W': grid[1] -= 1; break;
      }
      
      
      wb_motor_set_velocity(left_motor, 0.35 * MAX_SPEED);
      wb_motor_set_velocity(right_motor, 0.35 * MAX_SPEED);
      delay = 3751*0.98; //correction factor
      
      int start_clock = wb_robot_get_time() * 1000;
      int finish_clock = start_clock + delay;
      while (wb_robot_get_time() * 1000 < finish_clock && dsForwardVal < COLLISION_THRESHOLD) {
        dsForwardVal = wb_distance_sensor_get_value(ds_forward);
        wb_robot_step(TIME_STEP);
      }
    } else if (commands[i] == 'L') {
      --headingIndex;
      if(headingIndex < 0) headingIndex +=4;
      IMU_next += PI/2;
      if(IMU_next > PI) IMU_next -= 2 * PI;  // loop to negative
      
      updateDirection(left_motor, right_motor, IMU, IMU_zero, &IMU_next);
    } else if (commands[i] == 'R') {
      ++headingIndex;
      if(headingIndex > 3) headingIndex -=4;
      IMU_next -= PI/2;
      if(IMU_next <= -PI) IMU_next += 2 * PI;  // loop to positive
      
      updateDirection(left_motor, right_motor, IMU, IMU_zero, &IMU_next);
      
    } else {
      printf("Unknown command\n");
      delay = 0;
    }
    
    // Stuck in time step loop until delay complete
      
    ++i;
  };
  
  
  // Finish and cleanup
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  
  dsForwardVal = wb_distance_sensor_get_value(ds_forward);
  dsLeftVal = wb_distance_sensor_get_value(ds_left);
  dsRightVal = wb_distance_sensor_get_value(ds_right);
  printf("Current command: %d - %c, ", i, commands[i]);
  printf("Grid: (%d, %d), ", grid[0], grid[1]);
  printf("Heading: %c, ", heading[headingIndex]);
  printf("Left Wall: ");
  if(dsLeftVal > WALL_SENSOR_THRESHOLD) {
    printf("Y, ");
  } else {
    printf("N, ");
  }
  printf("Front Wall: ");
  if(dsForwardVal > WALL_SENSOR_THRESHOLD) {
    printf("Y, ");
  } else {
    printf("N, ");
  }
  printf("Right Wall: ");
  if(dsRightVal > WALL_SENSOR_THRESHOLD) {
    printf("Y\n");
  } else {
    printf("N\n");
  }
  
  fclose(fptr);
  printf("Finished - Path plan executed\n");
  wb_robot_cleanup();

  return 0;
}

//void updateSpeed(WbDeviceTag left_motor, WbDeviceTag right_motor, 

void updateDirection(WbDeviceTag left_motor, WbDeviceTag right_motor, WbDeviceTag IMU, const double IMU_zero, double *IMU_next) {
  double IMU_current = 0;
  const double *IMU_val;
  double IMU_diff = 0;
  double K = TURN_GAIN;
  double speed = 0;
  
  while (wb_robot_step(TIME_STEP) != -1) {
    // Implement a P controller
    IMU_val = wb_inertial_unit_get_roll_pitch_yaw(IMU);
    IMU_current = *(IMU_val + 2) - IMU_zero;
    IMU_diff = *IMU_next - IMU_current;
    if(IMU_diff > PI) IMU_diff -= 2 * PI;
    if(IMU_diff < -PI) IMU_diff += 2 * PI;
    //printf("IMU: %f %f %f\n", IMU_current, IMU_next, IMU_diff);
    
    speed = IMU_diff * K * MAX_SPEED;
    if(speed > MAX_SPEED / 2) speed = MAX_SPEED / 2;
    if(speed < -MAX_SPEED / 2) speed = -MAX_SPEED / 2;
    if(speed > 0 && speed < 0.05) speed = 0.05;
    if(speed < 0 && speed > -0.05) speed = -0.05;
    
    wb_motor_set_velocity(left_motor, -speed);
    wb_motor_set_velocity(right_motor, speed);
    if(IMU_diff < 0.001 && IMU_diff > -0.001) break;
  }
}







