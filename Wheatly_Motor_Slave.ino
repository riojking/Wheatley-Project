/*
 * Wheatley Motor Slave
 * Rio King
 * 
 * This code listens for data over I2C and drives
 * both motor controllers based on that data.
 */

#include <Wire.h>
//set the port to connect to
const int I2C_Port = 1;

//the scale to smooth by
const double SMOOTH_MOD = 4.0;
const double TRACK_SMOOTH_MOD = 3.0;

//set the pins for the motor controllers
const int forward_backward_motor_pin_1 = 2;
const int forward_backward_motor_pin_2 = 7;
const int forward_backward_motor_enable_pin = 6;

const int up_down_motor_pin_1 = 11;
const int up_down_motor_pin_2 = 12;
const int up_down_motor_enable_pin = 10;

const int left_right_motor_pin_1 = 13;
const int left_right_motor_pin_2 = 8;
const int left_right_motor_enable_pin = 9;

//motor speed variable
double forward_backward_motor_speed = 0;
double up_down_motor_speed = 0;
double left_right_motor_speed = 0;

//target motor speeds
int target_forward_backward_motor_speed = 0;
int target_up_down_motor_speed = 0;
int target_left_right_motor_speed = 0;

//if the remote is on, smooth
bool smoothing = false;

void setup() {
  Wire.begin(I2C_Port);                // join i2c bus with address #1
  Wire.onReceive(receive_data); // set it so when data is recieved, the receive_data function is run
  Serial.begin(9600);           // start serial for output
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receive_data(int howMany) {

  //read if the remote is on or not
  smoothing = Wire.read();
  //read the target positions, scaled back to between -255 and 255
  target_left_right_motor_speed = (Wire.read() - 127) * 2;
  target_up_down_motor_speed = (Wire.read() - 127) * 2;
  target_forward_backward_motor_speed = (Wire.read() - 127) * 2;

  ///////////// TEMPORARY /////////////////////////////
  
  if(smoothing == false){
    Serial.print(target_left_right_motor_speed);
    Serial.print(" ");
    Serial.print(target_up_down_motor_speed);
    Serial.print(" ");
    Serial.println(target_forward_backward_motor_speed);
  }
  
}

//accelerate and decelerate motion
void smooth_motor(int* target, double* current, double smooth_mod){
  //if the current speed needs to accelerate
  if(*target > *current){
    //add the smooth mod scale to the current speed
    *current += smooth_mod;
    //if the speed is now past the target speed
    if(*current > *target){
      //set the speed equal to the target speed
      *current = *target;
    }
  }
  else if(*target < *current){
    //subtract the smooth scale from the current speed
    *current -= smooth_mod;
    //if the speed is now past the target speed
    if(*current < *target){
      //set them equal
      *current = *target;
    }
  }
}

//write the speed to the given pins
void write_to_motor(int pin_1, int pin_2, int enable_pin, int PMW_Speed)
{
  //if the speed is positive, spin the motor forward
  if(PMW_Speed > 0){
    digitalWrite(pin_1, HIGH);
    digitalWrite(pin_2, LOW);
    analogWrite(enable_pin, PMW_Speed);
  }
  //if the speed is negative, spin the motor backward
  else if(PMW_Speed < 0){
    digitalWrite(pin_1, LOW);
    digitalWrite(pin_2, HIGH);
    analogWrite(enable_pin, -PMW_Speed);
  }
  //if the speed is 0, brake the motor
  else{
    digitalWrite(pin_1, LOW);
    digitalWrite(pin_2, LOW);
    analogWrite(enable_pin, 0);
  }
}

void loop() {

  //declaired in setup, if data is recieved, update target speed varibles  

  //smooth the acceleration if smoothing is on
  if(smoothing == true){
    smooth_motor(&target_forward_backward_motor_speed, &forward_backward_motor_speed,TRACK_SMOOTH_MOD);
    smooth_motor(&target_up_down_motor_speed, &up_down_motor_speed,SMOOTH_MOD);
    smooth_motor(&target_left_right_motor_speed, &left_right_motor_speed,SMOOTH_MOD);
  }
  //if smoothing isn't on, set the speeds to the target speeds
  else{
    forward_backward_motor_speed = target_forward_backward_motor_speed;
    up_down_motor_speed = target_up_down_motor_speed;
    left_right_motor_speed = target_left_right_motor_speed;
  }
  //write to the motors
  write_to_motor(forward_backward_motor_pin_1, forward_backward_motor_pin_2, forward_backward_motor_enable_pin, forward_backward_motor_speed);
  write_to_motor(up_down_motor_pin_1, up_down_motor_pin_2, up_down_motor_enable_pin, up_down_motor_speed);
  write_to_motor(left_right_motor_pin_1, left_right_motor_pin_2, left_right_motor_enable_pin, left_right_motor_speed);
  
}
