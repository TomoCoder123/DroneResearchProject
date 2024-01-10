/*
 * File:          sgbaNoRSSI.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>

/*
 * You may want to add macros here.
 */

#include <math.h>
#include <stdio.h>
#include<time.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/camera.h>
#include <webots/receiver.h>

#include <webots/distance_sensor.h>
#include <sys/time.h>
#include "../junk/pid_controller.h"
#include "../junk/wallfollowing_multiranger.h"

//static variables only used for initialization, initialized in init gradient bug loop controller
static int timesp = 0;
static bool first_run = false;
static float ref_distance_from_wall = 0;
static float max_speed = 0.5;
static bool priority = false;
uint8_t rssi_threshold = 55;// normal batteries 50/52/53/53 bigger bbatteries 55/57/59
uint8_t rssi_collision_threshold = 46; // normal batteris 43/45/45/46 bigger batteries 48/50/52, limit before collision
static int start_start_time;
// statemachine functions
static float wanted_angle = 0;
static int own_id;
static bool outbound = true;
static int state_wf = 1;
static int near_drone_id = 10;
static int state_start_time;
static CommandVel_t cmdVel = {0,0,0}; //maybe wrong `:(

static void set_id(){

 const char *name = wb_robot_get_name();
 own_id = name[9];

}

#define deg2rad(angleDegrees) (angleDegrees * (float)M_PI / 180.0f)

// Converts radians to degrees.
#define rad2deg(angleRadians) (angleRadians * 180.0f / (float)M_PI)

static float wraptopi(float number) //converts angles to radians
{
  if (number > (float)M_PI) {
    return (number - (float)(2 * M_PI));
  } else if (number < (float)(-1 * M_PI)) {
    return (number + (float)(2 * M_PI));
  } else {
    return (number);
  }

}

uint8_t ss2rssi(double signal_strength){
  uint8_t rssi = (uint8_t)round(70-(7*sqrt(signal_strength)));
  return rssi;
}

bool set_priority(){
  if(near_drone_id > own_id){ //
    priority = false;
  }
  else {
    priority = true;
  }
}
uint8_t maxValue(uint8_t myArray[], int size)
{

  int i;
  uint8_t maxValue = myArray[0]; //return the maxvalue of the array

  for (i = 1; i < size; ++i) {
    if (myArray[i] > maxValue) {
      maxValue = myArray[i];
    }
  }
  return maxValue;
}

int transition(int new_state){
  int t =  get_time();
  state_start_time = t; //sets a state variable to the new state and get the time of the transition.

  return new_state;

}
int get_time(){

 double time = wb_robot_get_time();
//do stuff

 return time;
}
static bool logicIsCloseTo(float real_value, float checked_value, float margin) //wraptopi(current_heading - wanted_angle), 0, 0.1f
{
  if (real_value > checked_value - margin && real_value < checked_value + margin) {
    return true;
  } else {
    return false;
  }
}
static float fillHeadingArray(uint8_t *correct_heading_array, float rssi_heading, int diff_rssi, int max_meters){ //correct_heading_array, heading_rssi, diff_rssi, 4
  //Heading array of action choices
  static float heading_array[8] = { -135.0f, -90.0f, -45.0f, 0.0f, 45.0f, 90.0f, 135.0f, 180.0f};
  float rssi_heading_deg = rad2deg(rssi_heading);
  for (int it = 0; it < 8; it++) {
    if ((rssi_heading_deg >= heading_array[it] - 22.5f && rssi_heading_deg < heading_array[it] + 22.5f && it != 7) || (//if rssi heading is between the range
          it == 7 && (rssi_heading_deg >= heading_array[it] - 22.5f || rssi_heading_deg < -135.0f - 22.5f))) {
            uint8_t temp_value_forward = correct_heading_array[it];
            uint8_t temp_value_backward = correct_heading_array[(it + 4) % 8];  //change to the direction that is oppsotei
          // if gradient is good, increment the array corresponding to the current heading and decrement the exact opposite
          if (diff_rssi > 0) {
            correct_heading_array[it] = temp_value_forward + 1; //(uint8_t)abs(diff_rssi);
            if (temp_value_backward > 0) {
              correct_heading_array[(it + 4) % 8] = temp_value_backward - 1;  //(uint8_t)abs(diff_rssi);
            }
          }
                      // if gradient is bad, decrement the array corresponding to the current heading and increment the exact opposite
          else if (diff_rssi<0){
            if (temp_value_forward > 0) {
              correct_heading_array[it] = temp_value_forward - 1;  //(uint8_t)abs(diff_rssi);
            }
            correct_heading_array[(it + 4) % 8] = temp_value_backward + 1; //(uint8_t)abs(diff_rssi);

            }


        }
  }
  // degrading function
    //    If one of the arrays goes over maximum amount of points (meters), then decrement all values
if (maxValue(correct_heading_array, 8) > max_meters) {
  for (int it = 0; it < 8; it++) {
    if (correct_heading_array[it] > 0) {
      correct_heading_array[it] = correct_heading_array[it] - 1;
    }
  }
}

// Calculate heading where the beacon might be
int count = 0;
float y_part = 0, x_part = 0;
for (int it = 0; it < 8; it++) {
  if (correct_heading_array[it] > 0) {
    x_part += (float)correct_heading_array[it] * (float)cos(heading_array[it] * (float)M_PI / 180.0f); //heading array values represent angles in degrees, and are used here for their components
    y_part += (float)correct_heading_array[it] * (float)sin(heading_array[it] * (float)M_PI / 180.0f);

    //sum += heading_array[it];
    count = count + correct_heading_array[it]; //sum all the previous heading movements to obtain direction to return to.

  }
}
float wanted_angle_return = 0;
if (count != 0) {
  wanted_angle_return = atan2(y_part / (float)count, x_part / (float)count); //returns the expected angle fo the beacon
}


return wanted_angle_return;
}

// Command functions
static void commandTurn(float *vel_w, float max_rate)
{
  *vel_w = max_rate;
}
void init_gradient_bug_loop_controller(float new_ref_distance_from_wall, float max_speed_ref,
                                       float begin_wanted_heading, int initState) //same as wall follower init but with initState parameter.
{
  printf("arrived in igblc\n");
  ref_distance_from_wall = new_ref_distance_from_wall;
  max_speed = max_speed_ref;
  wanted_angle = begin_wanted_heading;
  //init_median_filter_i(&medFiltdiffRssi,101);
  first_run = true;
  printf("successfully initialized gblc\n");
}

int gradient_bug_loop_controller(float *vel_x, float *vel_y, double *vel_w, float *rssi_angle, int *state_wallfollowing,
                                 float front_range, float left_range, float right_range, float back_range,
                                 float current_heading, float current_pos_x, float current_pos_y, uint8_t rssi_beacon,
                                 uint8_t rssi_inter, float rssi_angle_inter, bool priority, bool outbound)//variables we have:  vel_x, vel_y, vel_w?, state_wallfolllowing, front_range, left_range, right_range, back_range, current_heading, current_pos_x, current_pos_y
{
  printf("---------SGBA---------\n");

  printf("arrived in gblc\n");


  // Initalize static variables
  static int state = 2;
  //static float previous_heading = 0;
  static float wanted_angle_dir = 0;
  static float pos_x_hit = 0;
  static float pos_y_hit = 0;
  static float pos_x_sample = 0;
  static float pos_y_sample = 0;
  //static float pos_x_move = 0;
  //static float pos_y_move = 0;
  static bool overwrite_and_reverse_direction = false;
  static float direction = 1;
  static bool cannot_go_to_goal = false;
  static uint8_t prev_rssi = 150;
  static int diff_rssi = 0;
  static bool rssi_sample_reset = false;
  static float heading_rssi = 0;
  static uint8_t correct_heading_array[8] = {0};

  static bool first_time_inbound = true;
  static float wanted_angle_hit = 0;

  /***********************************************************
   * State definitions
   ***********************************************************/
  // 1 = forward
  // 2 = rotate_to_goal
  // 3 = wall_following
  // 4 = move out of way

  /***********************************************************
   * Handle state transitions
   ***********************************************************/

  if (first_run) {

    wanted_angle_dir = wraptopi(current_heading - wanted_angle); // to determine the direction when turning to goal

    overwrite_and_reverse_direction = false;
    state = 2; //enter rotate to goal state
    state_start_time = get_time();
    first_run = false;
    wallFollowerInit(0.25, 0.3);

  }

  if (first_time_inbound) {
    wraptopi(wanted_angle - 3.14f);
    wanted_angle_dir = wraptopi(current_heading - wanted_angle);
    state = transition(2);
    first_time_inbound = false;
  }

  if(state==1){
    printf("Transition state 1\n");

  // if looping is detected, reverse direction (only on outbound)
  if (overwrite_and_reverse_direction) {
    direction = -1.0f * direction;
    overwrite_and_reverse_direction = false;
  } else {
    if (left_range < right_range && left_range < 2.0f) { //if something is within 2, and left is closer than right, move right
      direction = -1.0f;
    } else if (left_range > right_range && right_range < 2.0f) { //default is right
      direction = 1.0f;

    } else if (left_range > 2.0f && right_range > 2.0f) {
      direction = 1.0f;
    }

  }

  pos_x_hit = current_pos_x;    //
  pos_y_hit = current_pos_y;    //
  wanted_angle_hit = wanted_angle;  //

  wallFollowerInit(0.25, 0.3);

  for (int i = 0; i < 8; i++) { correct_heading_array[i] = 0; } //

  state = transition(3); //wall_following //
  }
  else if(state==2){
    printf("Transition state 2\n");

    // check if heading is close to the preferred_angle

    bool goal_check = logicIsCloseTo(wraptopi(current_heading - wanted_angle), 0, 0.1f);

    if (front_range < ref_distance_from_wall + 0.2f) {

      cannot_go_to_goal =  true;
      wallFollowerInit(0.25, 0.3);


      state = transition(3); //wall_following
  }
    if (goal_check) {

      state = transition(1); //forward

    }
  }

  else if(state==3){
    printf("Transition state 3\n");
    printf("StateWF %d\n",state_wf);
    // if another drone is close and there is no right of way, move out of the way
    printf("TIMESP: %d ", timesp);
    timesp++;

    if(timesp>1200){
      outbound = false;
  }
  printf(outbound ? "Outbound: true\n" : "Outbound: false\n");

    if (priority == false && rssi_inter < rssi_threshold) { //if received rssi from drones is less than the threshold, ?

      if (outbound) {

        if ((rssi_angle_inter < 0 && wanted_angle < 0) || (rssi_angle_inter > 0 && wanted_angle > 0)) {
          wanted_angle = -1 * wanted_angle;
          wanted_angle_dir = wraptopi(current_heading - wanted_angle);
          //state= transition(2);
        }
      }
      if (rssi_inter < rssi_collision_threshold) { //if theres a collision, do move out the way
        state = transition(4);

      }
    }
      // If going forward with wall following and cannot_go_to_goal bool is still on
      //    turn it off!
      if (state_wf == 5 && cannot_go_to_goal) {
        cannot_go_to_goal  = false;
      }



      // Check if the goal is reachable from the current point of view of the agent
      float bearing_to_goal = wraptopi(wanted_angle - current_heading);
      bool goal_check_WF = false;

      if (direction == -1) {
        goal_check_WF = (bearing_to_goal < 0 && bearing_to_goal > -1.57f);
      //  printf("bearing_to_goal: 0 > %f > -1.57\n", bearing_to_goal);

      } else {
        goal_check_WF = (bearing_to_goal > 0 && bearing_to_goal < 1.57f);
      //  printf("bearing_to_goal: 0 < %f < 1.57\n", bearing_to_goal);

      }
      // printf(goal_check_WF ? "Goal check true,\n " : "Goal check false,\n ");


      // Check if bug went into a looping while wall following,
      //    if so, then forse the reverse direction predical.
      float rel_x_loop = current_pos_x - pos_x_hit;   //  diff_rssi = (int)prev_rssi - (int)rssi_beacon;
      float rel_y_loop = current_pos_y - pos_y_hit;
      float loop_angle = wraptopi(atan2(rel_y_loop, rel_x_loop));

      //if(outbound)
      //{


      if (fabs(wraptopi(wanted_angle_hit + 3.14f - loop_angle)) < 1.0) {
        overwrite_and_reverse_direction = true;
      } else {
       // TODO: check following line causes deadlocks!!!!
        //overwrite_and_reverse_direction = false; // this didn't really work...
      }
      //}else{
      //  overwrite_and_reverse_direction = false;
      //}

      // if during wallfollowing, agent goes around wall, and heading is close to rssi _angle
      //      got to rotate to goal
      // TODO check if the cannot go to goal still does anything useful...

      bool frontrangecheck = front_range > ref_distance_from_wall + 0.4f;
      // printf(frontrangecheck ? "Front range check true, " : "Front range check false, ");
      // printf("front_range: %f > distance: %f\n", front_range, (ref_distance_from_wall + 0.4f));
      // printf(cannot_go_to_goal ? "cannot go to goal: true\n" : "cannot go to goal false\n");

      if ((state_wf == 6 || state_wf == 7) && goal_check_WF && front_range > ref_distance_from_wall + 0.4f
          && !cannot_go_to_goal) {


        wanted_angle_dir = wraptopi(current_heading - wanted_angle); // to determine the direction when turning to goal
        state = transition(2); //rotate_to_goal
      }

      // If going straight
      //    determine by the gradient of the crazyradio what the approx direction is.
      if (state_wf == 5) {


        if (!outbound) {
          // Reset sample gathering
          if (rssi_sample_reset) {
            pos_x_sample = current_pos_x;
            pos_y_sample = current_pos_y;
            rssi_sample_reset = false;
            prev_rssi = rssi_beacon;
          }


          // if the crazyflie traveled for 1 meter, than measure if it went into the right path
          float rel_x_sample = current_pos_x - pos_x_sample;
          float rel_y_sample = current_pos_y - pos_y_sample;
          float distance = sqrt(rel_x_sample * rel_x_sample + rel_y_sample * rel_y_sample);
          printf("distance: %f", distance);
          if (distance > 1.0f) {
            rssi_sample_reset = true;
            heading_rssi = current_heading;
            int diff_rssi_unf = (int)prev_rssi - (int)rssi_beacon;

            //rssi already gets filtered at the radio_link.c
            diff_rssi = diff_rssi_unf;

            // Estimate the angle to the beacon
            wanted_angle = fillHeadingArray(correct_heading_array, heading_rssi, diff_rssi, 4); //fill heading only happens on inbound.
            printf("wanted_angle: %f", wanted_angle);

            // for simulation
            //printf("wanted_angle %f, heading_rssi %f, diff_rssi, %d \n",wanted_angle,heading_rssi,diff_rssi);
            //for(int it=0;it<8;it++)printf("%d, ",correct_heading_array[it]);printf("\n");
          }
        }

      } else {
        rssi_sample_reset = true;
      }
    }

  else if (state == 4) {
    printf("Transition state 4\n");
    //MOVE_OUT_OF_WAY
   // once the drone has gone by, rotate to goal
   if (rssi_inter >= rssi_collision_threshold) {

     state = transition(2); //rotate_to_goal
   }
 }
 /***********************************************************
  * Handle state actions
  ***********************************************************/
  float temp_vel_x = 0;
  float temp_vel_y = 0;
  float temp_vel_w = 0;

  if (state == 1) {
    printf("Action state 1\n");
        //FORWARD
    // stop moving if there is another drone in the way
    // forward max speed
    if (left_range < ref_distance_from_wall) {
      temp_vel_y = -0.2f;
    }
    if (right_range < ref_distance_from_wall) {
      temp_vel_y = 0.2f;
    }
    temp_vel_x = 0.5;
    }

   else  if (state == 2) {  //ROTATE_TO_GOAL
    // rotate to goal, determined on the sign
    printf("Action state 2\n");

    if (wanted_angle_dir < 0) {

      commandTurn(&temp_vel_w, 0.5);

    } else {

      commandTurn(&temp_vel_w, -0.5);
    }


  } else  if (state == 3) {
    printf("Action state 3\n");

    //WALL_FOLLOWING
    //Get the values from the wallfollowing
    if (direction == -1) {
      //printf("front_range %f, left_range %f\n",front_range, left_range);
      state_wf = wallFollower(&cmdVel, front_range, left_range, current_heading, direction, wb_robot_get_time()); //float *vel_x, float *vel_y, float *vel_w, float front_range,
      //printf("StateWFII %d\n",state_wf);

      //float side_range, float current_heading, int direction_turn
    } else {
      //printf("front_range %f, right_range %f\n",front_range, right_range);

      state_wf = wallFollower(&cmdVel, front_range, right_range, current_heading, direction, wb_robot_get_time());
      //printf("StateWFII %d\n",state_wf);

    }
  } else if (state == 4) {      //MOVE_AWAY
    printf("Action state 4\n");

    float save_distance = 0.7f;
    if (left_range < save_distance) {
      temp_vel_y = temp_vel_y - 0.5f;
    }
    if (right_range < save_distance) {
      temp_vel_y = temp_vel_y + 0.5f;
    }
    if (front_range < save_distance) {
      temp_vel_x = temp_vel_x - 0.5f;
    }
    if (back_range < save_distance) {
      temp_vel_x = temp_vel_x + 0.5f;
    }

  }

  *rssi_angle = wanted_angle;

  *vel_x = temp_vel_x;
  *vel_y = temp_vel_y;
  *vel_w = temp_vel_w;

  printf("state: %d\n", state);
  printf("----------------------\n");
  return state;

}

int main(int argc, char **argv) {

  printf("arrived in main \n");
  wb_robot_init();

  int timestep = (int)wb_robot_get_basic_time_step(); //set timestep 32 milliseconds
  float rssi_angle = 0;

  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, timestep);

  WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
  wb_motor_set_position(m1_motor, INFINITY);
  wb_motor_set_velocity(m1_motor, -1.0);
  WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
  wb_motor_set_position(m2_motor, INFINITY);
  wb_motor_set_velocity(m2_motor, 1.0);
  WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
  wb_motor_set_position(m3_motor, INFINITY);
  wb_motor_set_velocity(m3_motor, -1.0);
  WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");
  wb_motor_set_position(m4_motor, INFINITY);
  wb_motor_set_velocity(m4_motor, 1.0);

  // Initialize Sensors
  //implement RSSI later, flow deck?
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag range_front = wb_robot_get_device("range_front");
  wb_distance_sensor_enable(range_front, timestep);
  WbDeviceTag range_left = wb_robot_get_device("range_left");
  wb_distance_sensor_enable(range_left, timestep);
  WbDeviceTag range_back = wb_robot_get_device("range_back");
  wb_distance_sensor_enable(range_back, timestep);
  WbDeviceTag range_right = wb_robot_get_device("range_right");
  wb_distance_sensor_enable(range_right, timestep);


  //wait 2 seconds
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0)
      break;
  }

  // Initialize state variables
  ActualState_t actualState = {0}; //states from pid_controller, roll, pitch, yaw_rate, altitude vx, vy ,
  DesiredState_t desiredState = {0}; //same thing as actual state
  //initialize

  double pastXGlobal =0;
  double pastYGlobal=0; //most recent x and y position
  double past_time = wb_robot_get_time(); //get webots time
  GainsPID_t gainsPID;// double kp_att_rp; double kd_att_rp;  double kp_att_y; double kd_att_y double kp_vel_xy;...
    //Kp is a proportional component, Ki is an integral component, and Kd is a derivative component.
  gainsPID.kp_att_y = 1;
  gainsPID.kd_att_y = 0.5;
  gainsPID.kp_att_rp =0.5;
  gainsPID.kd_att_rp = 0.1;
  gainsPID.kp_vel_xy = 2;
  gainsPID.kd_vel_xy = 0.5;
  gainsPID.kp_z = 10;
  gainsPID.ki_z = 50;
  gainsPID.kd_z = 5;
  init_pid_attitude_fixed_height_controller(); //sets error variables to 0
  float cmd_vel_y;
  float cmd_vel_x;
  float cmd_vel_w;

  // Initialize struct for motor power
  MotorPower_t motorPower;

  printf("Take off!\n");
  init_gradient_bug_loop_controller(0.5, 0.3, 0, 0);
  uint8_t beacon_rssi = 0;

  while (wb_robot_step(timestep) != -1) {
    printf("-------------------LOOP------------\n");

     printf("arrived in loop\n");
     const double dt = wb_robot_get_time() - past_time; //change in time
     actualState.roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
     printf("roll: %f ", actualState.roll);
     actualState.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
     printf("pitch: %f ", actualState.pitch);
     actualState.yaw_rate = wb_gyro_get_values(gyro)[2];
     printf("yaw_rate: %f ", actualState.yaw_rate);
     actualState.altitude = wb_gps_get_values(gps)[2];
     printf("altitude: %f", actualState.altitude);



     double xGlobal= wb_gps_get_values(gps)[0];
     double vxGlobal = (xGlobal - pastXGlobal)/dt; //x velocity
     double yGlobal = wb_gps_get_values(gps)[1];
     double vyGlobal = (yGlobal - pastYGlobal)/dt;
     printf("xGlobal: %f ", xGlobal);
     printf("yGlobal: %f ", yGlobal);
     printf("vxGlobal: %f ", vxGlobal);
     printf("vyGlobal: %f", yGlobal);

    // Get body fixed velocities
    float actualYaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    double cosyaw = cos(actualYaw);
    printf("actualYaw: %f\n", actualYaw);
    double sinyaw = sin(actualYaw);
    actualState.vx = vxGlobal * cosyaw + vyGlobal * sinyaw;  //get both components and find velocity, also I guess y is actually z.
    actualState.vy = - vxGlobal * sinyaw + vyGlobal * cosyaw;

    desiredState.roll = 0;   //ideal positions and orientations at each state?
    desiredState.pitch = 0;
    desiredState.vx = 0;
    desiredState.vy = 0;
    desiredState.yaw_rate = 0;
    desiredState.altitude = 0.7;

    float range_front_value = (float)wb_distance_sensor_get_value(range_front)/1000.0f;
    printf("real Front Range: %f\n",range_front_value );
    float range_right_value = (float)wb_distance_sensor_get_value(range_right)/1000.0f; //get range values
    float range_left_value = (float)wb_distance_sensor_get_value(range_left)/1000.0f;
    float range_back_value = (float)wb_distance_sensor_get_value(range_back)/1000.0f;
    set_priority();
     // float cmdVelX; float cmdVelY; float cmdAngW;
      //used in following lines at the very end of wallFollower
      //commandVel->cmdVelX = cmdVelXTemp;
  //commandVel->cmdVelY = cmdVelYTemp;
  //commandVel->cmdAngW = cmdAngWTemp;
    //gradient_bug_loop_controller();
    printf("length: %d\n",wb_receiver_get_queue_length(receiver));
     if(wb_receiver_get_queue_length(receiver) > 0){

       const char *message = wb_receiver_get_data(receiver);
       const double *dir = wb_receiver_get_emitter_direction(receiver);
       double signal = wb_receiver_get_signal_strength(receiver);
       beacon_rssi = ss2rssi(signal);
       printf("received: -%s- (signal=%g translated signal = %u, dir=[%g %g %g])\n",
         message, signal,beacon_rssi, dir[0], dir[1], dir[2]);

        wb_receiver_next_packet(receiver);


     }

    gradient_bug_loop_controller(&cmd_vel_x, &cmd_vel_y, &cmd_vel_w, &rssi_angle, &state_wf,
                                    range_front_value, range_left_value , range_right_value, range_back_value,
                                    (actualYaw), xGlobal, yGlobal,beacon_rssi ,
                                    100, 100, priority, outbound);
    desiredState.yaw_rate = cmdVel.cmdAngW; //set the desired yaw rate to the command in wallfollowing multiranger.

    // PID velocity controller with fixed height
    //printf("CMDVEL X: %f\n", cmdVel.cmdVelX);

    desiredState.vy = cmdVel.cmdVelY;
    desiredState.vx = cmdVel.cmdVelX;

    printf("desired velocity: (x -> %f, y -> %f) desired yaw_rate: (%f)\n",desiredState.vx, desiredState.vy, desiredState.yaw_rate);
    //printf("Motor Power (1: %f, 2: %f, 3: %f, 4: %f )\n",motorPower.m1,motorPower.m2,motorPower.m3,motorPower.m4);


    pid_velocity_fixed_height_controller(actualState, &desiredState,
    gainsPID, dt, &motorPower);
    //printf("dt:%f\n",dt);
    //printf("actualState : (x -> %f, y -> %f) actual yaw_rate: (%f)\n",actualState.vx, actualState.vy, actualState.yaw_rate);

    printf("Motor Power (1: %f, 2: %f, 3: %f, 4: %f )\n",motorPower.m1,motorPower.m2,motorPower.m3,motorPower.m4);
    float mean = (motorPower.m1 + motorPower.m2 + motorPower.m3 + motorPower.m4)/4;
    printf("std: 1: %f, 2: %f, 3: %f, 4: %f )\n",(mean-motorPower.m1),(mean-motorPower.m2),(mean-motorPower.m3),(mean-motorPower.m4));
    // PID attitude controller with fixed height
    /*desiredState.roll = sidewaysDesired;
    desiredState.pitch = forwardDesired;
     pid_attitude_fixed_height_controller(actualState, &desiredState,
    gainsPID, dt, &motorPower);*/

    // Setting motorspeed
    wb_motor_set_velocity(m1_motor, - motorPower.m1);
    wb_motor_set_velocity(m2_motor, motorPower.m2);
    wb_motor_set_velocity(m3_motor, - motorPower.m3);
    wb_motor_set_velocity(m4_motor, motorPower.m4);

    // Save past time for next time step
    past_time = wb_robot_get_time();
    pastXGlobal = xGlobal;
    pastYGlobal = yGlobal;

  };

  wb_robot_cleanup();

  return 0;
}
