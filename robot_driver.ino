/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// Use the following line if you have a Leonardo or MKR1000
#define USE_USBCON

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <Dynamixel2Arduino.h>

//#include <MKRIMU.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;

const uint8_t LEFT = 1; //left motor
const uint8_t RIGHT = 0;  //right motor
const float DXL_PROTOCOL_VERSION = 2.0;

const int8_t DIR_LEFT = 1;
const int8_t DIR_RIGHT = -1;
const float WHEEL_RAD = 0.033;  //meters
const float WHEEL_BASE = 0.184; // meters

uint8_t ITER = 0;

float vel = 0.0;
float newvel = 0.0;
float w = 0.0;
float rpm_left = 0.0;
float rpm_right = 0.0;
float w_left = 0.0;
float w_right = 0.0;

float w_x = 0.0;
float w_y = 0.0;
float w_z = 0.0;

bool read_IMU = true;
bool drive_motor = true;

unsigned long timer = millis();

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

void cmdVelCallback(const geometry_msgs::Twist& twist){
  vel = twist.linear.x;
  w = twist.angular.z;
}

ros::NodeHandle nh;

geometry_msgs::Twist cmd_vel;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCallback);

geometry_msgs::Twist imu_msg;
ros::Publisher imu_reading("imu", &imu_msg);

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

int set_mode_velocity(int DXL_ID) {
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  dxl.torqueOn(DXL_ID);
}

int spin_motor(int DXL_ID, float RPM, int DIR) {
  if(RPM==0){
    dxl.ledOff(DXL_ID);
  }else{
    dxl.ledOn(DXL_ID);
  }
  dxl.setGoalVelocity(DXL_ID, RPM*DIR, UNIT_RPM);
}


float convert_w_to_rpm(float w){
  return 60*w/(2*PI);
}

void setup()
{
  DEBUG_SERIAL.begin(9600);
  
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  if (read_IMU){
    nh.advertise(imu_reading);
  }
  if (drive_motor){
    nh.subscribe(sub);

    // Set Dynamixel Baudrate and protocol version, must match current configuration of motors
    dxl.begin(57600);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    // Configure motors for velocity mode
    set_mode_velocity(LEFT);
    set_mode_velocity(RIGHT);
  }
  while(!nh.connected()){
    nh.spinOnce();
  }
  nh.loginfo("Connected to OpenRB-150");

  if (read_IMU){
    while (!bno.begin()){
      nh.loginfo("Failure connecting to IMU, trying again");
      nh.spinOnce();
    }
    nh.loginfo("IMU Connection Established");
    bno.setExtCrystalUse(true);
  }
}

void loop()
{
  while(!nh.connected()){
    spin_motor(LEFT, 0, DIR_LEFT);
    spin_motor(RIGHT, 0, DIR_RIGHT);
    nh.spinOnce();
  }
  if (drive_motor){
    w_left = (vel-(w*WHEEL_BASE)/2)/WHEEL_RAD;
    w_right = (vel+(w*WHEEL_BASE)/2)/WHEEL_RAD;
    rpm_left = convert_w_to_rpm(w_left);
    rpm_right = convert_w_to_rpm(w_right);
    spin_motor(LEFT, rpm_left, DIR_LEFT);
    spin_motor(RIGHT, rpm_right, DIR_RIGHT);
  }
  if (read_IMU){
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu_msg.angular.x = gyro.x();
    imu_msg.angular.y = gyro.y();
    imu_msg.angular.z = gyro.z();

    imu_reading.publish( &imu_msg);
  }
  
  nh.spinOnce();
  //delay(15);
}
