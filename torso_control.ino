#include <AccelStepperEncoder.h>
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <cb1_msgs/TorsoControl.h>
#include <sensor_msgs/JointState.h>

#define LIMIT 12
#define DIR 14
#define STEP 15
#define SLEEP 16
#define M2 17
#define M1 18
#define M0 19
#define ENABLE 20
#define STEPS_PER_MM 400
#define MAX_LIMIT 480
#define MIN_LIMIT 10
#define MAX_SPEED 2500 * 16
#define MAX_ACCEL 8000 * 16

AccelStepperEncoder stepper(1, STEP, DIR);
Encoder encoder(2, 1);

ros::NodeHandle_<ArduinoHardware, 6, 6, 150, 150> nh;

std_msgs::Float32 joint_state;
ros::Publisher joint_pub("torso_state", &joint_state);
long previousMillis = 0;
long interval = 50;

long target_pos_steps;


void command_cb( const std_msgs::Float32& cmd_msg) {
  // convert target in mm to steps
  float target = cmd_msg.data * 1000.0; // convert to mm

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

ros::Subscriber<std_msgs::Float32> cmd_sub("torso_cmd", command_cb);

void home_stepper() {
  // enable stepper
  digitalWrite(SLEEP, HIGH);

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

ros::Subscriber<std_msgs::Empty> home_sub("torso_home", home_cb);

void setup() {
  pinMode(SLEEP, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(LIMIT, INPUT_PULLUP);

  digitalWrite(SLEEP, HIGH);
  digitalWrite(M0, LOW); // 1/16th stepping
  digitalWrite(M1, LOW); // 1/16th stepping
  digitalWrite(M2, HIGH); // 1/16th stepping

  target_pos_steps = 0;

  stepper.addEncoder(&encoder, 1.3333f);
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(MAX_ACCEL);
  stepper.setMinPulseWidth(2);
  home_stepper();
  //nh.getHardware()->setBaud(250000);
  nh.initNode();
  nh.subscribe(home_sub);
  nh.subscribe(cmd_sub);
  nh.advertise(joint_pub);


}

void loop()
{
  if (stepper.distanceToGo() == 0) {
    digitalWrite(13, HIGH);
    digitalWrite(SLEEP, LOW);
  } else {
    digitalWrite(13, LOW);
    digitalWrite(SLEEP, HIGH);
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    joint_state.data = (stepper.currentPosition() / STEPS_PER_MM) / 1000.0;
    joint_pub.publish(&joint_state);
  }
  nh.spinOnce();
  stepper.run();
  stepper.correctDeviation();
}
