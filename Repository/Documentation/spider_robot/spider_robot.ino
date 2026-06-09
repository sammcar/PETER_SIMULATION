/* -----------------------------------------------------------------------------
  - Project: Remote control Crawling robot
  - Author:  panerqiang@sunfounder.com
  - Date:  2015/1/27
   -----------------------------------------------------------------------------
  - Overview
  - This project was written for the Crawling robot desigened by Sunfounder.
    This version of the robot has 4 legs, and each leg is driven by 3 servos.
  This robot is driven by a Ardunio Nano Board with an expansion Board.
  We recommend that you view the product documentation before using.
  - Request
  - This project requires some library files, which you can find in the head of
    this file. Make sure you have installed these files.
  - How to
  - Before use,you must to adjust the robot,in order to make it more accurate.
    - Adjustment operation
    1.uncomment ADJUST, make and run
    2.comment ADJUST, uncomment VERIFY
    3.measure real sites and set to real_site[4][3], make and run
    4.comment VERIFY, make and run
  The document describes in detail how to operate.
   ---------------------------------------------------------------------------*/

// modified by Regis for spider project, 2015-09-26
// add remote control by HC-06 bluetooth module

// modified by Anuchit for spider project, 2015-11-28
// add remote control with android app for bluetooth spp
// add test robot function for command mode
// add sonar to measure distance between robot and obstacle
// add free walk mode use ultrasonic to avoid obstacle like vaccuum robot


/* Includes ------------------------------------------------------------------*/
//#include <Servo.h>    //to define and control servos
#include <FlexiTimer2.h>//to set a timer to manage all servos
// RegisHsu, remote control
#include <SerialCommand.h>
SerialCommand SCmd;   // The demo SerialCommand object

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
/* Servos --------------------------------------------------------------------*/
//define 12 servos for 4 legs
//Servo servo[4][3];
//define servos' ports
const int servo_pin[4][3] = { 
  {5, 6, 4},  //0
  {1, 2, 0},  //1
  {9, 10, 8}, //2
  {13, 14, 12}//3 
};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  170 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  595 // this is the 'maximum' pulse length count (out of 4096)
uint8_t servonum = 0;
typedef struct servos{
  int num;
  int dn90;
  int dn45;
  int d0;
  int d90;
  int d135;
}servos_t;

servos_t s_servos[16]={
  {0  , 145 ,  253  , 360 , 577 ,  -1 },
  {1  , 618 ,  502  , 385 , 155 ,  -1 },
  {2  , -1  ,  573  , 470 , 262 ,  155},
  
  {3  , -1  ,  -1   , -1  , -1  ,  -1 },
  
  {4  , 163 ,  267  , 370 , 590 ,  -1 },
  {5  , 175 ,  284  , 393 , 630 ,  -1 },
  {6  , -1  ,  155  , 257 , 475 ,  590},
  
  {7  , -1  ,  -1   , -1  , -1  ,  -1 },

  {8  , 145 ,  251  , 357 , 575 ,  -1 },
  {9  , 610 ,  503  , 395 , 187 ,  -1 },
  {10  , -1 ,  605  , 490 , 275 ,  160},
  
  {11  , -1  ,  -1   , -1  , -1  ,  -1 },
  
  {12  , 173 ,  282  , 390 , 610 ,  -1 },
  {13  , 163 ,  265  , 367 , 590 ,  -1 },
  {14  , -1  ,  165  , 270 , 487 ,  605},
  
  {15  , -1  ,  -1   , -1  , -1  ,  -1 }
};

///////////////////////////////////////////////////////////
//Define Arduino Pin for DC Motors
#define LEG0_IN1   2
#define LEG0_IN2   4
#define LEG0_EN    3
 
#define LEG1_IN1   7
#define LEG1_IN2   8
#define LEG1_EN    9
 
#define LEG2_IN1  12
#define LEG2_IN2  13
#define LEG2_EN   11

#define LEG3_IN1  14
#define LEG3_IN2  15
#define LEG3_EN   10
typedef struct {
  int EN;
  int IN1;
  int IN2; 
}m_leg_t;
m_leg_t M_LEG[4] = {
  {LEG0_EN, LEG0_IN1, LEG0_IN2},
  {LEG1_EN, LEG1_IN1, LEG1_IN2},
  {LEG2_EN, LEG2_IN1, LEG2_IN2},
  {LEG3_EN, LEG3_IN1, LEG3_IN2}
};
int v_c_alpha[4], v_c_beta[4], v_c_gamma[4];
int v_e_alpha[4], v_e_beta[4], v_e_gamma[4];
int v_mode;
int mode;
#define SPIDER  0
#define VEHICLE 1
#define VMODE_C 0
#define VMODE_N 1
#define VMODE_H 2
#define DIR_NO  0
#define DIR_CW  1
#define DIR_CCW 2
///////////////////////////////////////////////////////////
bool servo_service_en = true;
/* Size of the robot ---------------------------------------------------------*/
const float length_a = 54.45; //femur
const float length_b = 92.05; //tibia
const float length_c = 45.65; //coxa
const float length_side = 61.6; //body
const float z_absolute = 28;
/* Constants for movement ----------------------------------------------------*/
//const float z_default = 71.2127694+40, z_up = 71.2127694-40, z_boot = 71.2127694+40;// z_absolute;
const float z_default = length_b, z_up = 50, z_boot = length_b;// z_absolute;
//sqrt(pow(length_a + length_c, 2)/2)
const float x_default = sqrt(pow(length_a + length_c, 2)/2),  x_offset = 0;
const float y_start =  0,  y_step = 70;
const float y_default = x_default;
/*
// Size of the robot ---------------------------------------------------------
const float length_a = 55;
const float length_b = 77.5;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;
// Constants for movement ----------------------------------------------------
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;
const float y_default = x_default;
*/
/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];    //real-time coordinates of the end of each leg
volatile float site_expect[4][3]; //expected coordinates of the end of each leg
float temp_speed[4][3];   //each axis' speed, needs to be recalculated before each movement
float move_speed;     //movement speed
float speed_multiple = 1; //movement speed multiple
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter;      //+1/0.02s, for automatic rest
//functions' parameter
const float KEEP = 255;
//define PI for calculation
const float pi = 3.1415926;
/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

/*
  - setup function
   ---------------------------------------------------------------------------*/
void setup()
{
  //start serial for debug
  Serial.begin(9600);
  Serial.println("Robot starts initialization");
  pwm.begin(); 
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(100);
  // RegisHsu, remote control
  // Setup callbacks for SerialCommand commands
  // action command 0-6,
  // w 0 1: stand
  // w 0 0: sit
  // w 1 x: forward x step
  // w 2 x: back x step
  // w 3 x: right turn x step
  // w 4 x: left turn x step
  // w 5 x: hand shake x times
  // w 6 x: hand wave x times
  // Anuchit
  // w 7 0: sonar_mode
  // w 8 0: freewalk mode
  // w 9 0: leg init
  SCmd.addCommand("w", action_cmd);

  SCmd.setDefaultHandler(unrecognized);

  //initialize default parameter
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
   
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
    move_speed = 8;
  mode = SPIDER;
  //start servo service
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();
  Serial.println("Servo service started");
  //initialize servos
 // servo_attach();
  Serial.println("Servos initialized");
  Serial.println("Robot initialization Complete");
  pinMode(LEG0_IN1, OUTPUT);
  pinMode(LEG0_IN2, OUTPUT);
  pinMode(LEG0_EN, OUTPUT);
  pinMode(LEG1_IN1, OUTPUT);
  pinMode(LEG1_IN2, OUTPUT);
  pinMode(LEG1_EN, OUTPUT);
  pinMode(LEG2_IN1, OUTPUT);
  pinMode(LEG2_IN2, OUTPUT);
  pinMode(LEG2_EN, OUTPUT);
  pinMode(LEG3_IN1, OUTPUT);
  pinMode(LEG3_IN2, OUTPUT);
  pinMode(LEG3_EN, OUTPUT);

#if 0
// Initialize servos
  set_site(0, x_default, y_default, z_boot);
  set_site(1, x_default, y_default, z_boot);
  set_site(2, x_default, y_default, z_boot);
  set_site(3, x_default, y_default, z_boot);

while(1);

#endif
}

/*
  - loop function
   ---------------------------------------------------------------------------*/
void loop()
{
  SCmd.readSerial();
 // do_test();
}


void do_test(void)
{
  delay(1000);
  Serial.println("Stand");
  stand();
  delay(100);
  Serial.println("Step forward");
  step_forward(2);
  delay(100);
  Serial.println("Turn left");
  turn_left(2);
  delay(500);
  Serial.println("Step back");
  step_back(2);
  delay(500);
  Serial.println("Turn right");
  turn_right(2);
  delay(500);
  /*Serial.println("Hand wave");
  hand_wave(3);
  delay(2000);
  Serial.println("Hand shake");
  hand_shake(3);
  delay(2000);
  Serial.println("Body dance");
  body_dance(10);
  delay(2000);    
  Serial.println("Sit");
  sit();
  delay(5000);
  */
  vehicle_mode();
  delay(100);
  v_shiftleft();
  delay(1500);
  v_forwardright();
  delay(1500);
  v_shiftleft();
  delay(1500);
  v_backward();
  delay(1500);
  v_stop();
  delay(1000);
  v_rotateleft();
  delay(2000);
  v_rotateright();
  delay(2000);
  v_stop();
  delay(1000);
  vehicle_burst_nor();
  delay(500);
  v_forward();
  delay(2000);
  v_backward();
  delay(2000);
  v_stop();
  delay(1000);
  vehicle_center();
  vehicle_burst_hor();
  v_forward();
  delay(2000);
  v_backward();
  delay(2000);
  v_stop();
  delay(1000);
  spider_mode();
  delay(3000);
}

// RegisHsu
// w 0 1: stand
// w 0 0: sit
// w 1 x: forward x step
// w 2 x: back x step
// w 3 x: right turn x step
// w 4 x: left turn x step
// w 5 x: hand shake x times
// w 6 x: hand wave x times
// Anuchit
// w 7 0: sonar_mode
// w 8 0: freewalk mode
// w 9 0: leg init
#define W_STAND_SIT    0
#define W_FORWARD      1
#define W_BACKWARD     2
#define W_LEFT         3
#define W_RIGHT        4
#define W_SHAKE        5
#define W_WAVE         6
#define W_VEHICLE      7
#define W_VEHICLE_C    8
#define W_VEHICLE_N    9
#define W_VEHICLE_H    10
#define W_R_LEFT       11
#define W_R_RIGHT      12
#define W_F_LEFT       13
#define W_F_RIGHT      14
#define W_B_LEFT       15
#define W_B_RIGHT      16
#define W_SPIDER       17

void action_cmd(void)
{
  char *arg;
  int action_mode, n_step;
 // Serial.println("Action:");
  arg = SCmd.next();
  action_mode = atoi(arg);
  arg = SCmd.next();
  n_step = atoi(arg);
  Serial.end();
  switch (action_mode)
  {
    case W_FORWARD:
      Serial.println("Step forward");
      if(mode == SPIDER){
        if (!is_stand())
          stand();
        step_forward(n_step);
      }
      else
        v_forward();
      break;
    case W_BACKWARD:
      Serial.println("Step back");
      if(mode == SPIDER){      
        if (!is_stand())
          stand();
        step_back(n_step);
      }
      else
        v_backward();
      break;
    case W_LEFT:
     Serial.println("Turn left");
     if(mode == SPIDER){
        if (!is_stand())
          stand();
        turn_left(n_step);
      }
      else
        v_shiftleft();
     break;
    case W_RIGHT:
      Serial.println("Turn right");
      if(mode == SPIDER){
        if (!is_stand())
          stand();
        turn_right(n_step);
      }
      else 
        v_shiftright();
      break;
    case W_STAND_SIT:
      Serial.println("1:up,0:dn");
      if (n_step)
        stand();
      else
        sit();
      break;
    case W_SHAKE:
      Serial.println("Hand shake");
      hand_shake(n_step);
      break;
    case W_WAVE:
      Serial.println("Hand wave");
      hand_wave(n_step);
      break;
    case W_VEHICLE:
      Serial.println("Vehicle Mode");
      vehicle_mode();
      break;
    case W_VEHICLE_C:
      Serial.println("Vehicle Center");
      vehicle_center();
      break;
    case W_VEHICLE_N:
      Serial.println("Vehicle N");
      vehicle_burst_nor();
      break;
    case W_VEHICLE_H:
      Serial.println("Vehicle H");
      vehicle_burst_hor();
      break;
    case W_R_LEFT:
      Serial.println("Rotate Left");
      v_rotateleft();
      break;
    case W_R_RIGHT:
      Serial.println("Rotate Right");
      v_rotateright();
      break;
    case W_F_LEFT:
      Serial.println("Forward Left");
      v_forwardleft();
      break;
    case W_F_RIGHT:
      Serial.println("Forward Right");
      v_forwardright();
      break;
    case W_B_LEFT:
      Serial.println("Backward Left");
      v_backwardleft();
      break;
    case W_B_RIGHT:
      Serial.println("Backward Right");
      v_backwardright();
      break;
    case W_SPIDER:
      Serial.println("Spider Mode");
      spider_mode();
    default:
      Serial.println("Error");
      break;
  }
  Serial.begin(9600);
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println("What?");
}

/*
 * - legs init
 */

void legs_init(void){
  
  //initialize all servos
  move_speed = 8;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, 0, 90);
  }
  wait_all_reach();
}

/*
  - is_stand
   ---------------------------------------------------------------------------*/
bool is_stand(void)
{
  if (site_now[0][2] == z_default)
    return true;
  else
    return false;
}

/*
  - sit
  - blocking function
   ---------------------------------------------------------------------------*/
void sit(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

/*
  - stand
  - blocking function
   ---------------------------------------------------------------------------*/
void stand(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}



/*
  - spot turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_left(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}


/*
  - spot turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_right(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}


/*
  - go forward
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_forward(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go back
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_back(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

// add by RegisHsu
void body_left(int i)
{
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

void body_right(int i)
{
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

void hand_wave(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void hand_shake(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void head_up(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] - i);
  set_site(1, KEEP, KEEP, site_now[1][2] + i);
  set_site(2, KEEP, KEEP, site_now[2][2] - i);
  set_site(3, KEEP, KEEP, site_now[3][2] + i);
  wait_all_reach();
}

void head_down(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  wait_all_reach();
}

void body_dance(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  float body_dance_speed = 2;
  sit();
  move_speed = 1;
  set_site(0, x_default, y_default, KEEP);
  set_site(1, x_default, y_default, KEEP);
  set_site(2, x_default, y_default, KEEP);
  set_site(3, x_default, y_default, KEEP);
  wait_all_reach();
  //stand();
  set_site(0, x_default, y_default, z_default - 20);
  set_site(1, x_default, y_default, z_default - 20);
  set_site(2, x_default, y_default, z_default - 20);
  set_site(3, x_default, y_default, z_default - 20);
  wait_all_reach();
  move_speed = body_dance_speed;
  head_up(30);
  for (int j = 0; j < i; j++)
  {
    if (j > i / 4)
      move_speed = body_dance_speed * 2;
    if (j > i / 2)
      move_speed = body_dance_speed * 3;
    set_site(0, KEEP, y_default - 20, KEEP);
    set_site(1, KEEP, y_default + 20, KEEP);
    set_site(2, KEEP, y_default - 20, KEEP);
    set_site(3, KEEP, y_default + 20, KEEP);
    wait_all_reach();
    set_site(0, KEEP, y_default + 20, KEEP);
    set_site(1, KEEP, y_default - 20, KEEP);
    set_site(2, KEEP, y_default + 20, KEEP);
    set_site(3, KEEP, y_default - 20, KEEP);
    wait_all_reach();
  }
  move_speed = body_dance_speed;
  head_down(30);
}

/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/
void servo_service(void)
{
  if(!servo_service_en)
    return;
  sei();
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

   cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
   polar_to_servo(i, alpha, beta, gamma);
  }
  rest_counter++;
}

/*
  - set one of end points' expect site
  - this founction will set temp_speed[4][3] at same time
  - non - blocking function
   ---------------------------------------------------------------------------*/
void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}

/*
  - wait one of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_reach(int leg)
{
  while (1)
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break;
}

/*
  - wait all of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_all_reach(void)
{
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}

/*
  - trans site from cartesian to polar
  - mathematical model 2/2
   ---------------------------------------------------------------------------*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));  //L1
  v = w - length_c; //L1-coxa
  alpha = atan2(v, z) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

/*
  - trans site from polar to microservos
  - mathematical model map to fact
  - the errors saved in eeprom will be add
   ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  if (leg == 0)
  {
    alpha = 90 - alpha;
    beta = 180 - beta;
    gamma -= 45;
  }
  else if (leg == 1)
  {
    alpha = 90 - alpha;
    beta = 180 - beta;
    gamma = -gamma + 45;
  }
  else if (leg == 2)
  {
    alpha = 90 - alpha;
    beta = 180 - beta;
    gamma = -gamma + 45;
  }
  else if (leg == 3)
  {
    alpha = 90 - alpha;
    beta = 180 - beta;
    gamma -= 45;
  }
  set_servos(s_servos[servo_pin[leg][0]], alpha);
  set_servos(s_servos[servo_pin[leg][1]], beta);
  set_servos(s_servos[servo_pin[leg][2]], gamma);
}

void set_servos(servos_t servo, int degree)
{
  int in_min, in_max, out_min, out_max;
  if(degree<-90||degree>155)
  {
    Serial.print(servo.num);
    Serial.print(" d:");
    Serial.print(degree);
    Serial.println("fail");
    return;
  }
  if(degree<-45)
  {
    in_min = -90;
    in_max = -45;
    out_min = servo.dn90;
    out_max = servo.dn45;
  }
  else if(degree<0)
  {
    in_min = -45;
    in_max = 0;
    out_min = servo.dn45;
    out_max = servo.d0;
  }
  else if(degree<90)
  {
    in_min = 0;
    in_max = 90;
    out_min = servo.d0;
    out_max = servo.d90;
  }
  else if(degree<=155)
  {
    in_min = 90;
    in_max = 135;
    out_min = servo.d90;
    out_max = servo.d135;
  }
  if(out_min==-1||out_max==-1)
  {
    Serial.print(servo.num);
    Serial.print(" d:");
    Serial.print(degree);
    Serial.println("fail");
    return;
  }
  int pulsew = map(degree, in_min, in_max, out_min, out_max);
 /* Serial.print("servo num:");
  Serial.print(servo.num);
  Serial.print(" pulse width:");
  Serial.print(pulsew);
  Serial.print(" angle:");
  Serial.println(degree);
 */
  pwm.setPWM(servo.num, 0, pulsew);
}

void vehicle_mode()
{
  if(mode != SPIDER)
    return;
  set_site(0, x_default, y_default, z_boot);
  set_site(1, x_default, y_default, z_boot);
  set_site(2, x_default, y_default, z_boot);
  set_site(3, x_default, y_default, z_boot);
  wait_all_reach();
  servo_service_en = false;
  mode = VEHICLE;
  for(int i=0;i<4;i++)
  {
    v_c_alpha[i] = 0;
    v_c_beta[i] = 90;
    v_c_gamma[i] = 0;
    v_e_alpha[i] = v_c_alpha[i];
    v_e_beta[i] =  v_c_beta[i];
    v_e_gamma[i] = v_c_gamma[i];
  }
  vehicle_center();
}
void spider_mode()
{
  if(mode != VEHICLE)
    return;
  v_stop();
  if(v_mode!=VMODE_C)
    vehicle_center();
  for(int i=0;i<4;i++)
  {
    v_e_alpha[i] = 0;
    v_e_beta[i] =  90;
    v_e_gamma[i] = 0;
  }
  update_vehicle_servo();
  servo_service_en = true;
  mode = SPIDER;
}
 
void update_vehicle_servo()
{
  while(memcmp(v_c_alpha,v_e_alpha,sizeof(v_c_alpha))||memcmp(v_c_beta,v_e_beta,sizeof(v_c_beta))||memcmp(v_c_gamma,v_e_gamma,sizeof(v_c_gamma)))
  {
    for(int leg=0;leg<4;leg++)
    {
      if(v_c_alpha[leg]!=v_e_alpha[leg])
      {
        v_c_alpha[leg]>v_e_alpha[leg]? v_c_alpha[leg]--:v_c_alpha[leg]++;
        set_servos(s_servos[servo_pin[leg][0]], v_c_alpha[leg]);
      }
      if(v_c_beta[leg]!=v_e_beta[leg])
      {
        v_c_beta[leg]>v_e_beta[leg]? v_c_beta[leg]--:v_c_beta[leg]++;
        set_servos(s_servos[servo_pin[leg][1]], v_c_beta[leg]);
      }
      if(v_c_gamma[leg]!=v_e_gamma[leg])
      {
        v_c_gamma[leg]>v_e_gamma[leg]? v_c_gamma[leg]--:v_c_gamma[leg]++;
        set_servos(s_servos[servo_pin[leg][2]], v_c_gamma[leg]);
      }
    }
    delay(5);
  }
}

void vehicle_center()
{
  for(int i=0;i<4;i++)
  {
    v_e_alpha[i] = 30;
    v_e_beta[i] = -30;
    v_e_gamma[i] = 0;
  }    
  update_vehicle_servo();
  v_mode = VMODE_C;
}

void vehicle_burst_hor()
{
  for(int i=0;i<4;i++)
  {
    v_e_alpha[i] = 30;
    v_e_beta[i] = -30;
  } 
  v_e_gamma[0] = 45;
  v_e_gamma[1] = -45;
  v_e_gamma[2] = -45;
  v_e_gamma[3] = 45;
  update_vehicle_servo();
  v_mode = VMODE_H;
}

void vehicle_burst_nor()
{
  for(int i=0;i<4;i++)
  {
    v_e_alpha[i] = 30;
    v_e_beta[i] = -30;
  } 
  v_e_gamma[0] = -45;
  v_e_gamma[1] = 45;
  v_e_gamma[2] = 45;
  v_e_gamma[3] = -45;
  update_vehicle_servo();
  v_mode = VMODE_N;
}
void motor_LEG(int leg_num, int dir)
{
  m_leg_t leg = M_LEG[leg_num];
  digitalWrite(leg.IN1, LOW);
  digitalWrite(leg.IN2, LOW);
  if(dir==DIR_CW)
  {
    digitalWrite(leg.IN1, HIGH);
  }
  else if(dir==DIR_CCW)
  {
    digitalWrite(leg.IN2, HIGH);
  }
  if(dir!=DIR_NO)
    analogWrite(leg.EN, 255);
  else
    analogWrite(leg.EN, 0);
}

void v_stop()
{
  motor_LEG(0, DIR_NO);
  motor_LEG(1, DIR_NO);
  motor_LEG(2, DIR_NO);
  motor_LEG(3, DIR_NO);
}
void v_forward()
{
  switch(v_mode)
  {
    case VMODE_C:
    case VMODE_N:
      motor_LEG(0, DIR_CW);
      motor_LEG(1, DIR_CW);
      motor_LEG(2, DIR_CCW);
      motor_LEG(3, DIR_CCW);
      break;
    case VMODE_H:
      motor_LEG(0, DIR_CW);
      motor_LEG(1, DIR_CCW);
      motor_LEG(2, DIR_CW);
      motor_LEG(3, DIR_CCW);
      break;
      default:
      break;
  }
}
void v_backward()
{
  switch(v_mode)
  {
    case VMODE_C:
    case VMODE_N:
      motor_LEG(0, DIR_CCW);
      motor_LEG(1, DIR_CCW);
      motor_LEG(2, DIR_CW);
      motor_LEG(3, DIR_CW);
      break;
    case VMODE_H:
      motor_LEG(0, DIR_CCW);
      motor_LEG(1, DIR_CW);
      motor_LEG(2, DIR_CCW);
      motor_LEG(3, DIR_CW);
      break;
      default:
      break;
  }
}

void v_shiftleft()
{
  switch(v_mode)
  {
    case VMODE_C:
      motor_LEG(0, DIR_CW);
      motor_LEG(1, DIR_CCW);
      motor_LEG(2, DIR_CW);
      motor_LEG(3, DIR_CCW);
      break;
    case VMODE_N:
    case VMODE_H:
      default:
      break;
  }
}
void v_shiftright()
{
  switch(v_mode)
  {
    case VMODE_C:
      motor_LEG(0, DIR_CCW);
      motor_LEG(1, DIR_CW);
      motor_LEG(2, DIR_CCW);
      motor_LEG(3, DIR_CW);
      break;
    case VMODE_N:
    case VMODE_H:
      default:
      break;
  }
}
void v_rotateleft()
{
  switch(v_mode)
  {
    case VMODE_C:
      motor_LEG(0, DIR_CW);
      motor_LEG(1, DIR_CW);
      motor_LEG(2, DIR_CW);
      motor_LEG(3, DIR_CW);
      break;
    case VMODE_N:
    case VMODE_H:
      default:
      break;
  }
}
void v_rotateright()
{
  switch(v_mode)
  {
    case VMODE_C:
      motor_LEG(0, DIR_CCW);
      motor_LEG(1, DIR_CCW);
      motor_LEG(2, DIR_CCW);
      motor_LEG(3, DIR_CCW);
      break;
    case VMODE_N:
    case VMODE_H:
      default:
      break;
  }
}
void v_forwardleft()
{
  switch(v_mode)
  {
    case VMODE_C:
      motor_LEG(0, DIR_CW);
      motor_LEG(1, DIR_NO);
      motor_LEG(2, DIR_NO);
      motor_LEG(3, DIR_CCW);
      break;
    case VMODE_N:
    case VMODE_H:
      default:
      break;
  }
}
void v_forwardright()
{
  switch(v_mode)
  {
    case VMODE_C:
      motor_LEG(0, DIR_NO);
      motor_LEG(1, DIR_CW);
      motor_LEG(2, DIR_CCW);
      motor_LEG(3, DIR_NO);
      break;
    case VMODE_N:
    case VMODE_H:
      default:
      break;
  }
}
void v_backwardleft()
{
  switch(v_mode)
  {
    case VMODE_C:
      motor_LEG(0, DIR_NO);
      motor_LEG(1, DIR_CCW);
      motor_LEG(2, DIR_CW);
      motor_LEG(3, DIR_NO);
      break;
    case VMODE_N:
    case VMODE_H:
      default:
      break;
  }
}
void v_backwardright()
{
  switch(v_mode)
  {
    case VMODE_C:
      motor_LEG(0, DIR_CCW);
      motor_LEG(1, DIR_NO);
      motor_LEG(2, DIR_NO);
      motor_LEG(3, DIR_CW);
      break;
    case VMODE_N:
    case VMODE_H:
      default:
      break;
  }
}
