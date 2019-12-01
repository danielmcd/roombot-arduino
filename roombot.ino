// Roombot - Roomba interface and ROS driver
//
// License: GPL V2 (http://www.gnu.org/copyleft/gpl.html)
//
// Dependencies:
// Roomba Library for Arduino (http://www.airspayce.com/mikem/arduino/Roomba/Roomba-1.3.zip) (License GPL V2)
// ros_lib (License BSD)
//
// Hardware Configuration:
// 7-pin DIN connector plugged into Roomba serial port
// Roomba battery (1/2) and ground (6/7) pins are connected to input of power regulator board
// Power regulator is set to produce 9V output from Roomba's input battery voltage
// 9V output from power regulator feeds Arduino UNO Vin and Gnd pins
// Roomba Rx (3/RXD) pin is connected to base of PNP transistor
// PNP transistor collector is connected to Arduino GND
// PNP transistor emitter is connected to Arduino pin 7 (Rx)
// Roomba Tx (4/TXD) pin is connected to Arduino pin 6 (Tx)
// Arduino USB is connected to Raspberry Pi 3 for ROS Serial communication
// Arduino pin 9 connects to tilt servo control wire (for optinoal Pi camera)
// Servo position feedback connects to Arduino pin A0
//
// Additional Hardware:
// Raspberry Pi onboard provides serial interface to Arduino and WiFi interface to laptop
// Laptop provides developer console and joystick input for Roombot control
// Pi camera attached to Pi and mounted on tilt servo
// XV-11 LIDAR sensor attached to control board
// LIDAR control board attached to Pi by USB
// WiFi router
// Logitech UDB gamepad
//
// Installation:
// Configure Pi to automatically connect to the WiFi router
// Configure WiFi router DHCP to assign fixed IP address to the Pi - this is the IP address that the laptop will connect to
// Upload the Roombot program to the Arduino
// Install ROS on the Pi
// Create a local catkin workspace and install the roombot python code
// Configure init scripts to start roscore and the roombot nodes when the Pi boots
//
// Operation:
// Plug DIN cable into Roomba serial port
// Attach and turn on Pi battery
// Connect laptop to the WiFi router and set ROS_MASTER_URI to point to the Pi
// Run the joy_node on the laptop and plug in the gamepad
//
// Copyright 2019 Daniel McDonald

#define __AVR_ATmega168__
#include <ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <HardwareSerial.h>
#include <HardwareSerial_private.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Roomba.h>

/**
 * Global symbols defining the software-hardware binding
 */
// NOTE: normally you connect Rx->Tx and Tx->Rx; however with the Roomba class and following the
// documentation here (https://www.instructables.com/id/Web-controlled-Twittering-Roomba/) it only
// works with Arduino Rx->Roomba Rx and Arduino Tx->Roomba Tx.
#define ROOMBA_RXD 7  // Arduino Rx pin connected to Roomba Rx
#define ROOMBA_TXD 6  // Arduino Tx pin connected to Roomba Tx

#define TILT_SERVO_PIN 9

#define ROSSERIAL_BAUD_RATE 57600

#define ROS_TWIST_TOPIC "twist"       // geometry_msgs::Twist
// TODO replace this with the standard Joy message limited to 10 Hz
#define ROS_BUTTONS_TOPIC "buttons"   // std_msgs::String (expecting 10 space delimited numbers)

/**
 * The Roomba object requires a HardwareSerial instance, but we need to use the UNO's hardware serial
 * connection for the rosserial connection to the Raspberry Pi.  Therefore, this class provides an adapter
 * between SoftwareSerial and the HardwareSerial interface methods used by the Roomba class.
 * 
 * The stock implementation of HardwareSerial does not declare the begin method as virtual, so HardwareSerial.h
 * must be modified to add virtual to this method for the following adapter to work correctly.
 */
class SerialBridge : public HardwareSerial {
  private:
    SoftwareSerial *_softSerial; // calls to the HardwareSerial interface are delegated to this instance
    
  public:
    SerialBridge(SoftwareSerial *softSerial) : HardwareSerial(0, 0, 0, 0, 0, 0) {
      _softSerial = softSerial;
    }
    
    virtual void begin(unsigned long baud) {
      // NOTE: This requires changing the HardwareSerial.h header to declare `begin` virtual
      _softSerial->begin(baud);
    }
    
    virtual int available() {
      return _softSerial->available();
    }
    
    virtual int read() {
      return _softSerial->read();
    }
    
    virtual size_t write(uint8_t data) {
      return _softSerial->write(data);
    }
};

ros::NodeHandle nh;
char message[32];
int vel = 0;
int rad = 0;
#define tiltMax 180  // corresponds to lowest camera angle
#define tiltMin 50   // corresponds to highest camera angle
#define tiltDefault 110  // the initial camera angle
int tilt = 0;
int newTilt = tiltDefault;

void on_twist(const geometry_msgs::Twist& twist_msg) {
  vel = twist_msg.linear.y;
  rad = twist_msg.angular.z;
  float tiltAngle = min(70, max(twist_msg.angular.x, -30));
  newTilt = map(tiltAngle, -30, 70, tiltMax, tiltMin);
  sprintf(message, "vel=%d, rad=%d", vel, rad);
  nh.loginfo(message);
}

int b0 = 0;
int b1 = 0;
int b2 = 0;
int b3 = 0;
int b4 = 0;
int b5 = 0;
int b6 = 0;
int b7 = 0;
int b8 = 0;
int b9 = 0;
int b10 = 0;
int b11 = 0;

void on_buttons(const std_msgs::String& buttons_msg) {
  sscanf(buttons_msg.data, "%d %d %d %d %d %d %d %d %d %d %d %d", &b0, &b1, &b2, &b3, &b4, &b5, &b6, &b7, &b8, &b9, &b10, &b11);
}

ros::Subscriber<geometry_msgs::Twist> sub(ROS_TWIST_TOPIC, on_twist);
ros::Subscriber<std_msgs::String> sub2(ROS_BUTTONS_TOPIC, on_buttons);

SoftwareSerial roombaSerial(ROOMBA_RXD, ROOMBA_TXD); // RX, TX
SerialBridge serialBridge(&roombaSerial);
Roomba roomba(&serialBridge, Roomba::Baud115200);  //Create a roomba object at 115200 BPS, the default for 500 Series Roombas

Servo tiltServo;

void setup() {
  pinMode(A0, INPUT_PULLUP);
  tiltServo.attach(TILT_SERVO_PIN);
  
  Serial.begin(ROSSERIAL_BAUD_RATE);
  
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);

  roomba.reset();
  roomba.power();
  roomba.start();
  roomba.safeMode();
}

int color = 200;
int intensity = 100;
int colorDirection = 3;
int intensityDirection = 1;
boolean isVacuuming = false;
boolean waitForReleaseStart = false;
uint8_t bump = 0;
uint8_t wall = 0;

void loop() {
  roomba.getSensors(Roomba::SensorBumpsAndWheelDrops, &bump, 1);
  bump &= 1; // left bump sensor is not working, so check only the right one
  if (bump > 0) {
    //sprintf(message, "bump: %d", bump);
    //nh.loginfo(message);
    if (vel > 0 && !(rad == 1 || rad == -1)) {
      vel = 0;
    }
  }
/*
  roomba.getSensors(Roomba::SensorWall, &wall, 1);
  wall &= 1;
  if (wall > 0) {
    sprintf(message, "wall: %d", wall);
    nh.loginfo(message);
    if (vel > 0 && !(rad == 1 || rad == -1)) {
      vel /= 2;
    }
  }
*/

  nh.spinOnce();
  
  roomba.drive(vel, rad);
  
  if (newTilt != tilt) {
    tilt = newTilt;
    tiltServo.write(tilt);
  }
  
  if (b9 == 1 && waitForReleaseStart == false && isVacuuming == false) {
    isVacuuming = true;
    waitForReleaseStart = true;
    roomba.drivers(ROOMBA_MASK_SIDE_BRUSH | ROOMBA_MASK_VACUUM | ROOMBA_MASK_MAIN_BRUSH);
  }
  else if (b9 == 0 && waitForReleaseStart) {
    waitForReleaseStart = false;
  }
  else if (b9 == 1 && waitForReleaseStart == false && isVacuuming == true) {
    roomba.drivers(0);
    isVacuuming = false;
    waitForReleaseStart = true;
  }

  roomba.leds(ROOMBA_MASK_LED_PLAY, color, intensity);

  color += colorDirection;
  
  if (color > 255) {
    color = 255;
    colorDirection = -colorDirection;
  }
  if (color < 0) {
    color = 0;
    colorDirection = -colorDirection;
  }
  
  intensity += intensityDirection;
  
  if (intensity > 255) {
    intensity = 255;
    intensityDirection = -intensityDirection;
  }
  if (intensity < 0) {
    intensity = 0;
    intensityDirection = -intensityDirection;
  }
 
  delay(15); //No need to go faster. Roomba only checks its sensors every 15ms. Going faster will only slow the Roomba down.

}
