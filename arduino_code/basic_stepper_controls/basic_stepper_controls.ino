#include <Stepper.h>

#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

#define STEPPER_PIN_1 8
#define STEPPER_PIN_2 10
#define STEPPER_PIN_3 9
#define STEPPER_PIN_4 11

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, STEPPER_PIN_1, STEPPER_PIN_2, STEPPER_PIN_3, STEPPER_PIN_4);

void messageStepper(const std_msgs::Empty& toggle_msg) {
  Serial.print("Fuck yeah");
  myStepper.step(stepsPerRevolution);
  delay(500);
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageStepper);

void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(60);
  // initialize the serial port:
  Serial.begin(9600);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  Serial.print("Before spin");
  // step one revolution  in one direction:
  nh.spinOnce();
  Serial.print("After spin");
  delay(1);
}
