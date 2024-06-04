#include <AccelStepper.h>
#include <ros.h>

#include <arm_gamepad_controller/Gamepad.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

// Declare Pin Numbers
#define J1_VRX_PIN A4
#define J1_VRY_PIN A5
#define J1_BUTTON_PIN 37

#define J2_VRX_PIN A0
#define J2_VRY_PIN A1
#define J2_BUTTON_PIN 39

#define J3_VRX_PIN A2
#define J3_VRY_PIN A3
#define J3_BUTTON_PIN 41

#define AXIS_1_PIN_1 23
#define AXIS_1_PIN_2 22

#define AXIS_2_PIN_1 25
#define AXIS_2_PIN_2 24

#define AXIS_3_PIN_1 27
#define AXIS_3_PIN_2 26

#define AXIS_4_PIN_1 29
#define AXIS_4_PIN_2 28

#define AXIS_5_PIN_1 31
#define AXIS_5_PIN_2 30

#define AXIS_6_PIN_1 33
#define AXIS_6_PIN_2 32

// Determines wheter the robot is in electromagnet mode or gripper mode
// For consistency, 0 will denote electromagnet mode and 1 will denote gripper mode
#define ROBOT_MODE_TOGGLE_PIN 0

#define ELECTROMAG_PIN 36

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution

// Initialize AccelStepper for the 6 motors
AccelStepper axis1(AccelStepper::DRIVER, AXIS_1_PIN_1, AXIS_1_PIN_2);
AccelStepper axis2(AccelStepper::DRIVER, AXIS_2_PIN_1, AXIS_2_PIN_2);
AccelStepper axis3(AccelStepper::DRIVER, AXIS_3_PIN_1, AXIS_3_PIN_2);
AccelStepper axis4(AccelStepper::DRIVER, AXIS_4_PIN_1, AXIS_4_PIN_2);
AccelStepper axis5(AccelStepper::DRIVER, AXIS_5_PIN_1, AXIS_5_PIN_2);
AccelStepper axis6(AccelStepper::DRIVER, AXIS_6_PIN_1, AXIS_6_PIN_2);

// Initialize vars for speed and controls
float av1, av2, av3, av4, av5, av6;
int J1_xVal, J1_yVal, J2_xVal, J2_yVal, J3_xVal, J3_yVal;
bool J1_button, J2_button, J3_button;

// Threshold to prevent the robot from acting from slight deviations
float threshold = 0.4;

// Method callbacks for each axis
void messageAxis1 (const std_msgs::Float64& control_msg) {
  float p = control_msg.data;

  float step_vel = p / 2 * 3.1415 * stepsPerRevolution;
  
  if(p > threshold)
    av1 = 50;
  else if(p < -threshold)
    av1 = -50;  
  else
    av1 = 0;
}

void messageAxis2 (const std_msgs::Float64& control_msg) {
  float p = control_msg.data;

  float step_vel = p / 2 * 3.1415 * stepsPerRevolution;
  
  if(p > threshold)
    av2 = 50;
  else if(p < -threshold)
    av2 = -50;  
  else
    av2 = 0;
}

void messageAxis3 (const std_msgs::Float64& control_msg) {
  float p = control_msg.data;

  float step_vel = p / 2 * 3.1415 * stepsPerRevolution;
  
  if(p > threshold)
    av3 = 50;
  else if(p < -threshold)
    av3 = -50;  
  else
    av3 = 0;
}

void messageAxis4 (const std_msgs::Float64& control_msg) {
  float p = control_msg.data;

  float step_vel = p / 2 * 3.1415 * stepsPerRevolution;
  
  if(p > threshold)
    av4 = 50;
  else if(p < -threshold)
    av4 = -50;  
  else
    av4 = 0;
}

void messageAxis5 (const std_msgs::Float64& control_msg) {
  float p = control_msg.data;

  float step_vel = p / 2 * 3.1415 * stepsPerRevolution;
  
  if(p > threshold)
    av5 = 50;
  else if(p < -threshold)
    av5 = -50;  
  else
    av5 = 0;
}

void messageAxis6 (const std_msgs::Float64& control_msg) {
  float p = control_msg.data;

  float step_vel = p / 2 * 3.1415 * stepsPerRevolution;
  
  if(p > threshold)
    av6 = 50;
  else if(p < -threshold)
    av6 = -50;  
  else
    av6 = 0;
}

bool electromag_toggle = 0;
void electromag_callback (const std_msgs::Bool& toggle) {
  if (electromag_toggle) {
    digitalWrite(ELECTROMAG_PIN, LOW);
  } else {
    digitalWrite(ELECTROMAG_PIN, HIGH);
  }
  electromag_toggle = !electromag_toggle;
  delay(200);
}

ros::Subscriber<std_msgs::Float64> sub1("/arm_axis_1_controller/command", &messageAxis1);
ros::Subscriber<std_msgs::Float64> sub2("/arm_axis_2_controller/command", &messageAxis2);
ros::Subscriber<std_msgs::Float64> sub3("/arm_axis_3_controller/command", &messageAxis3);
ros::Subscriber<std_msgs::Float64> sub4("/arm_axis_4_controller/command", &messageAxis4);
ros::Subscriber<std_msgs::Float64> sub5("/arm_axis_5_controller/command", &messageAxis5);
ros::Subscriber<std_msgs::Float64> sub6("/arm_axis_6_controller/command", &messageAxis6);

// ros::Subscriber<std_msgs::Float64> gripper_sub("/arm_gripper/command", &gripper_callback);
ros::Subscriber<std_msgs::Bool> electromag_sub("/arm_electromagnet/command", &electromag_callback);

arm_gamepad_controller::Gamepad gamepad_state;
ros::Publisher gamepad_state_pub("/gamepad_state", &gamepad_state);

// Calibration values for joysticks
int j1x_cal, j1y_cal, j2x_cal, j2y_cal, j3x_cal, j3y_cal;

void setup() {
  // initialize the serial port:
  Serial.begin(9600);

  pinMode(J1_BUTTON_PIN, INPUT_PULLUP);
  pinMode(J2_BUTTON_PIN, INPUT_PULLUP);
  pinMode(J3_BUTTON_PIN, INPUT_PULLUP);

  pinMode(ELECTROMAG_PIN, OUTPUT);

  axis1.setCurrentPosition(0);
  axis1.setMaxSpeed(60);
  axis1.setAcceleration(10000);

  axis2.setCurrentPosition(0);
  axis2.setMaxSpeed(60);
  axis2.setAcceleration(10000);

  axis3.setCurrentPosition(0);
  axis3.setMaxSpeed(60);
  axis3.setAcceleration(10000);

  axis4.setCurrentPosition(0);
  axis4.setMaxSpeed(60);
  axis4.setAcceleration(10000);

  axis5.setCurrentPosition(0);
  axis5.setMaxSpeed(60);
  axis5.setAcceleration(10000);

  axis6.setCurrentPosition(0);
  axis6.setMaxSpeed(60);
  axis6.setAcceleration(10000);

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  nh.subscribe(sub6);
  nh.subscribe(electromag_sub);

  j1x_cal = analogRead(J1_VRX_PIN);
  j1y_cal = analogRead(J1_VRY_PIN);
  j2x_cal = analogRead(J2_VRX_PIN);
  j2y_cal = analogRead(J2_VRY_PIN);
  j3x_cal = analogRead(J3_VRX_PIN);
  j3y_cal = analogRead(J3_VRY_PIN);

//  int robot_state = digitalRead(ROBOT_MODE_TOGGLE_PIN);
//  if (robot_state == 0) {
//    nh.subscribe(electromag_sub);
//  } else {
//    nh.subscribe(gripper_sub);
//  }

  nh.advertise(gamepad_state_pub);
}

void loop() {
  // step one revolution  in one direction:
  axis1.setSpeed(av1);
  axis1.runSpeed();

  axis2.setSpeed(av2);
  axis2.runSpeed();

  axis3.setSpeed(av3);
  axis3.runSpeed();

  axis4.setSpeed(av4);
  axis4.runSpeed();

  axis5.setSpeed(av5);
  axis5.runSpeed();

  axis6.setSpeed(av6);
  axis6.runSpeed();

  J1_xVal = analogRead(J1_VRX_PIN);
  J1_yVal = analogRead(J1_VRY_PIN);
  J1_button = digitalRead(J1_BUTTON_PIN);

  J2_xVal = analogRead(J2_VRX_PIN);
  J2_yVal = analogRead(J2_VRY_PIN);
  J2_button = digitalRead(J2_BUTTON_PIN);

  J3_xVal = analogRead(J3_VRX_PIN);
  J3_yVal = analogRead(J3_VRY_PIN);
  J3_button = digitalRead(J3_BUTTON_PIN);

  gamepad_state.j1_vrx = J1_xVal - j1x_cal;
  gamepad_state.j1_vry = J1_yVal - j1y_cal;
  gamepad_state.j1_button = J1_button;

  gamepad_state.j2_vrx = J2_xVal - j2x_cal;
  gamepad_state.j2_vry = J2_yVal - j2y_cal;
  gamepad_state.j2_button = J2_button;

  gamepad_state.j3_vrx = J3_xVal - j3x_cal;
  gamepad_state.j3_vry = J3_yVal - j3y_cal;
  gamepad_state.j3_button = J3_button;

  gamepad_state_pub.publish( &gamepad_state );

  Serial.print("x1 = ");
  Serial.print(J1_xVal);
  Serial.print(", y1 = ");
  Serial.print(J1_yVal);
  Serial.print(", button1 = ");
  Serial.print(J1_button);

  Serial.print("x2 = ");
  Serial.print(J2_xVal);
  Serial.print(", y2 = ");
  Serial.print(J2_yVal);
  Serial.print(", button2 = ");
  Serial.print(J2_button);

  Serial.print("x3 = ");
  Serial.print(J3_xVal);
  Serial.print(", y3 = ");
  Serial.print(J3_yVal);
  Serial.print(", button3 = ");
  Serial.println(J3_button);

  nh.spinOnce();

  delay(10);
}
