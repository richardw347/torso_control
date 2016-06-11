#include <AccelStepper.h>
#include "uStepperEncoder.h"
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

#define LIMIT 12
#define STEPS_PER_ROT 400
#define STEPS_PER_MM 50
#define MAX_LIMIT 480
#define MIN_LIMIT 10
#define MAX_SPEED 6000
#define MAX_ACCEL 8000

AccelStepper stepper(1, STEP, DIR);
uStepperEncoder encoder;
ros::NodeHandle_<ArduinoHardware, 6, 6, 150, 150> nh;

std_msgs::Float32 joint_state;
ros::Publisher joint_pub("torso_state", &joint_state);
long previousMillis = 0;
long interval = 50;
long target_pos_steps;


void command_cb( const std_msgs::Float32& cmd_msg) {
  // convert target in mm to steps
  float target = cmd_msg.data * 1000.0; // convert to mm
  
  //float acceleration = cmd_msg.acceleration * 1000.0 * STEPS_PER_MM;

  if (target > MAX_LIMIT) {
    target = MAX_LIMIT;
  }
  if (target < MIN_LIMIT) {
    target = MIN_LIMIT;
  }
  //if (acceleration > MAX_ACCEL){
  //  acceleration = MAX_ACCEL;
  //}

  target_pos_steps = target * STEPS_PER_MM;
  stepper.moveTo(target_pos_steps);
  //stepper.setAcceleration(acceleration);
}

ros::Subscriber<std_msgs::Float32> home_sub("torso_cmd", command_cb);

void home_stepper() {
  // enable stepper
  
  // find limit
  stepper.setAcceleration(10000 * 16);
  stepper.moveTo(-240000);
  while (1) {
    stepper.run();
    if (!digitalRead(LIMIT)) {
      stepper.stop();
      break;
    }
  }
  delay(100);
  stepper.setCurrentPosition(0);
  stepper.moveTo(2 * STEPS_PER_MM);
  while (stepper.distanceToGo() > 0) {
    stepper.run();
  }
  stepper.setMaxSpeed(100 * 16);
  stepper.moveTo(-240000);
  while (1) {
    stepper.run();
    if (!digitalRead(LIMIT)) {
      stepper.stop();
      break;
    }
  }
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(MAX_ACCEL);
  stepper.moveTo(MIN_LIMIT * STEPS_PER_MM);
}

void home_cb(const std_msgs::Empty& emp) {
  home_stepper();
}

ros::Subscriber<std_msgs::Empty> sub("torso_home", home_cb);

void setup() {
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);
  //pinMode(LIMIT, INPUT_PULLUP);
  digitalWrite(ENA, LOW);

  target_pos_steps = 0;
 
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(MAX_ACCEL);
  //stepper.setMinPulseWidth(2);
  //home_stepper();
  nh.getHardware()->setBaud(250000);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(home_sub);
  nh.advertise(joint_pub);
  //encoder.setHome();
}

void loop()
{
  if (stepper.distanceToGo() == 0) {
    digitalWrite(13, HIGH);
    digitalWrite(ENA, HIGH);
  } else {
    digitalWrite(13, LOW);
    digitalWrite(ENA, LOW);
  }
  float rotations = -encoder.getAngleMoved() / 360.0;
  long encoder_steps = rotations * STEPS_PER_ROT;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    joint_state.data = (encoder_steps / STEPS_PER_MM) / 1000.0;
    joint_pub.publish(&joint_state);
  }
  nh.spinOnce();
  
  stepper.setCurrentPositionOnly(encoder_steps);
  stepper.run();
  //stepper.correctDeviation();
}
