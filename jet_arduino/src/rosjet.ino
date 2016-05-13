#include "DualVNH5019MotorShield.h"
#include "NewPing.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include "Wire.h"

/*
 * CONSTANTS
 */
//Motor Constants
#define MAX_SPEED 400
#define BRAKE_POWER 400

//Sonar Constants
#define SONAR_NUM 3
#define MAX_DISTANCE 200
#define PING_INTERVAL 30

//Motor Current Constants
#define CURRENT_INTERVAL 30

//Encoder Constants
#define ENCODER_INTERVAL 20

//Accel/Gyro Constants
#define MPU_ADDR 0x68
#define MPU_INTERVAL 50

/*
 * GLOBAL VARIABLES
 */
 
//Sonar
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(53, 51, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(49, 47, MAX_DISTANCE),
  NewPing(45, 43, MAX_DISTANCE)
};
std_msgs::Int16 sonar_distance[SONAR_NUM];

ros::Publisher sonar_pub[SONAR_NUM] = {
  ros::Publisher("arduino/sonar_1", &sonar_distance[0]),
  ros::Publisher("arduino/sonar_2", &sonar_distance[1]),
  ros::Publisher("arduino/sonar_3", &sonar_distance[2])
};


//Motor Current
unsigned long currentTimer;
std_msgs::Int16 motor_right_current, motor_left_current;
ros::Publisher motor_right_current_pub("arduino/motor_right_current", &motor_right_current);
ros::Publisher motor_left_current_pub("arduino/motor_left_current", &motor_left_current);

//Motors
DualVNH5019MotorShield md;
void motor_right_speed_cb(const std_msgs::Int16 &cmd_msg);
void motor_left_speed_cb(const std_msgs::Int16 &cmd_msg);
ros::Subscriber<std_msgs::Int16> motor_right_speed_sub("arduino/motor_right_speed", motor_right_speed_cb);
ros::Subscriber<std_msgs::Int16> motor_left_speed_sub("arduino/motor_left_speed", motor_left_speed_cb);

//Ros NodeHandler
ros::NodeHandle  nh;

//Encoders
unsigned long encoderTimer;
std_msgs::Int16 encoder_left_value, encoder_right_value;
ros::Publisher encoder_left_pub("arduino/encoder_left_value", &encoder_left_value);
ros::Publisher encoder_right_pub("arduino/encoder_right_value", &encoder_right_value);

//Accel/Gyro Values
int16_t Tmp;
unsigned long mpuTimer;
geometry_msgs::Twist accel;
geometry_msgs::Twist gyro;
ros::Publisher accel_pub("arduino/accel", &accel);
ros::Publisher gyro_pub("arduino/gyro", &gyro);

/*
 * CALLBACKS
 */
void motor_right_speed_cb(const std_msgs::Int16 &cmd_msg) {
    md.setM2Speed(cmd_msg.data);
    if (cmd_msg.data == 0)
      md.setM2Brake(BRAKE_POWER);
}

void motor_left_speed_cb(const std_msgs::Int16 &cmd_msg) {
    md.setM1Speed(cmd_msg.data);
    if (cmd_msg.data == 0)
      md.setM1Brake(BRAKE_POWER);
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer()) 
    sonar_distance[currentSensor].data = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void encoder_left_cb() {
  encoder_left_value.data++;
}

void encoder_right_cb() {
  encoder_right_value.data++;
}

void readMPU() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR,14,true);  // request a total of 14 registers
    accel.linear.x = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    accel.linear.y = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    accel.linear.z = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gyro.angular.x = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gyro.angular.y = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gyro.angular.z = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    gyro_pub.publish(&gyro);
    accel_pub.publish(&accel);
}
void setup() {
  md.init();
  nh.initNode();

  nh.subscribe(motor_right_speed_sub);
  nh.subscribe(motor_left_speed_sub);

  nh.advertise(motor_right_current_pub);
  nh.advertise(motor_left_current_pub);
  
  nh.advertise(encoder_left_pub);
  nh.advertise(encoder_right_pub);
  
  nh.advertise(accel_pub);
  nh.advertise(gyro_pub);
  
  for(uint8_t i = 0; i < SONAR_NUM; i++) {
    nh.advertise(sonar_pub[i]);
  }
  
  currentTimer = millis() + 75;
  
  encoderTimer = millis() + 75;

  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    
  mpuTimer = millis() + 90;
  
  attachInterrupt(5, encoder_left_cb, RISING); //digital pin 18
  attachInterrupt(4, encoder_right_cb, RISING); //digital pin 19
  
  encoder_left_value.data = 0;
  encoder_right_value.data = 0;
  
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void loop() {  
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors
  
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      sonar_pub[i].publish(&sonar_distance[i]);
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      sonar_distance[currentSensor].data = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  
  if (millis() >= currentTimer) {
    motor_right_current.data = md.getM2CurrentMilliamps();
    motor_left_current.data = md.getM1CurrentMilliamps();
  
    motor_right_current_pub.publish(&motor_right_current);
    motor_left_current_pub.publish(&motor_left_current);
    currentTimer += CURRENT_INTERVAL;
  }
  
  if (millis() >= encoderTimer) {
    encoder_left_pub.publish(&encoder_left_value);
    encoder_right_pub.publish(&encoder_right_value);
    encoder_left_value.data = 0;
    encoder_right_value.data = 0;
    encoderTimer += ENCODER_INTERVAL;
  }
   
  if (millis() >= mpuTimer) {
      readMPU();
      mpuTimer += MPU_INTERVAL;
  }
  nh.spinOnce();
}
