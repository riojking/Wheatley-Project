/*
 * Wheatley Master Arduino
 * Rio King
 * 
 * This code runs on the master arduino. It reads inputs from the remote
 * and sends data over I2C to the two slave arduinos and the servo board.
 * This also controls the automations that take place when the remote is
 * turned off.
 * 
 */

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_PWMServoDriver.h>
//not sure what this next bit does
#ifdef __AVR__
  #include <avr/power.h>
#endif

//define the motor driver
Adafruit_PWMServoDriver ServoShield = Adafruit_PWMServoDriver();

//define the neopixel ring
const int NEOPIXEL_PIN = 12;
Adafruit_NeoPixel neoRing = Adafruit_NeoPixel(24, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
const int EYE_LED_PIN = 4;

//define the pins for the remote input
const int channel_1_pin = 11;
const int channel_2_pin = 10;
const int channel_3_pin = 9;
const int channel_4_pin = 6;
const int channel_5_pin = 5;
const int channel_6_pin = 3;

//define potentiometer pin, value, max value and min value
const int pot_pin = A0;
int pot_value = 0;
const int potMax = 420;
const int potMin = 100;

//tweak for auto
const int POT_BUFFER = 50;
const int up_down_full_speed_distance = 200;

// the adress for the slave arduinos
const int MOTOR_ARDUINO = 1;
const int SOUND_ARDUINO = 2;

//define the variables for the input chanels
int channel_1_value;
int channel_2_value;
int channel_3_value;
int channel_4_value;
int channel_5_value;
int channel_6_value;

//define the motor speeds
int up_down_motor_speed = 0;
int left_right_motor_speed = 0;
int forward_backward_motor_speed = 0;

//define pins for the four eye movement Servos
const int LeftRightEyeServoPin = 0;
const int UpDownEyeServoPin = 1;

const int TopEyelidServoPin = 4;
const int BottomEyelidServoPin = 5;

//the center point for each eye servo
int LeftRightEyeServoCenterValue = 110;
int UpDownEyeServoCenterValue = 115;

/*Eyelid positions:
 * Bottom Eyelid, all down - 115
 * Bottom Eyelid, all up - 
 * Top Eyelid, all down - 30
 * Top Eyelid, all up - 
 */
int BottomEyelidServoCenterValue = 95;
int TopEyelidServoCenterValue = 70;

//how far away from the center each eye servo is
//Left / down is negitive
//Right / up is positive
int LeftRightEyeServoValue = 0;
int UpDownEyeServoValue = 0;

int BottomEyelidServoValue = 0;
int TopEyelidServoValue = 0;

//how far to move each servo from its center
//these affect the ranges of motion
const int EYE_SERVO_MOVE_DISTANCE = 40;
const int EYELID_SERVO_MOVE_DISTANCE = 40;

//used to smmooth servo motion
const float SERVO_SMOOTH_FACTOR = 0.5;

//define numbers for the different sounds (the same numbers are defined in the sound slave code)
enum {SILENT,UP_HERE,NOT_MORON,JUMP_IN_PIT,BOXES_WITH_LEGS,DONT_LOOK,DONT_PANIC,NOT_DEAD,DONT_WATCH,TOTAL_SOUND_NUMBER};
//define the current sound to play
int current_sound = SILENT;
//the sound that the nob on the remote is currently pointing twoards
int target_sound = SILENT;
int previous_target_sound = SILENT;
//the amound of time that the nob has been pointing there
unsigned long int nob_time = millis();

//if the remote is on
bool remote_on = false;

//write to a servo
void write_to_servo(int channel, float degree)
{
  //define the min and max pwm values
  const int SERVO_MIN = 150;
  const int SERVO_MAX = 600;
  //map the degree value to a pwm value
  int pwm_value = map(degree, 0, 180, SERVO_MIN, SERVO_MAX);
  //constain the values
  constrain(pwm_value, SERVO_MIN, SERVO_MAX);
  //write to the servo shield
  ServoShield.setPWM(channel, 0, pwm_value);
  
}

//controll the neo pixel ring
//for now, just light up blue
void write_to_neo_ring(int brightness)
{
  //record if the on/off state changed
  static int previousBrightness = 0;
  if(previousBrightness != brightness){
    previousBrightness = brightness;
    
    //set the blue brightness of each LED the given brightness
    for(int i = 0; i < neoRing.numPixels(); ++i){
      neoRing.setPixelColor(i,0,0,brightness);
    }
    neoRing.show();

    //if the neoring is off, turn the center light off, otherwise keep it on
    if(brightness == 0){
      digitalWrite(EYE_LED_PIN, LOW);
    }
    else{
      digitalWrite(EYE_LED_PIN, HIGH);
    }
  }
}

//smooth servo motion
//moveTime is how long it has already been moving
float smooth_servo(float start, float finish)
{
  //if the servo is already close to the finish value, don't move it to prevent buzz
  if(abs(start-finish) <= 2.0){
    return start;
  }
  //get the distance that the servo needs to travel
  int distance = finish - start;
  //move the servo some of the way
  return start + distance * SERVO_SMOOTH_FACTOR;
  
}

//put each servo at its set value
void update_eye_servos()
{

  //actual values for the eye servos
  static float ActualLRValue = LeftRightEyeServoCenterValue;
  static float ActualUDValue = UpDownEyeServoCenterValue;
  static float ActualTEyelidValue = TopEyelidServoCenterValue;
  static float ActualBEyelidValue = BottomEyelidServoCenterValue;
  
  //get smoothed motion
  ActualLRValue = smooth_servo(ActualLRValue, LeftRightEyeServoCenterValue + LeftRightEyeServoValue);
  ActualUDValue = smooth_servo(ActualUDValue, UpDownEyeServoCenterValue + UpDownEyeServoValue);
  ActualTEyelidValue = smooth_servo(ActualTEyelidValue, TopEyelidServoCenterValue + TopEyelidServoValue);
  ActualBEyelidValue = smooth_servo(ActualBEyelidValue, BottomEyelidServoCenterValue + BottomEyelidServoValue);  
  
  //move the eye arround
  write_to_servo(LeftRightEyeServoPin, ActualLRValue);
  write_to_servo(UpDownEyeServoPin, ActualUDValue);
  
  //move the eyelids
  write_to_servo(TopEyelidServoPin, ActualTEyelidValue);
  write_to_servo(BottomEyelidServoPin, ActualBEyelidValue);
}

//update the remote channel values
void update_channels()
{
  //pulse in channel 1
  channel_1_value = pulseIn(channel_1_pin, HIGH, 30000);
  //if the remote is off, set all values to 0
  if(channel_1_value == 0){
    //set remote on to false
    remote_on = false;
    channel_2_value = 0;
    channel_3_value = 0;
    channel_4_value = 0;
    channel_5_value = 0;
    channel_6_value = 0;
  }
  //if the remote is on
  else{
    //set the remote on to true
    remote_on = true;
    //pulse in the other chanels
    channel_2_value = pulseIn(channel_2_pin, HIGH, 30000);
    channel_3_value = pulseIn(channel_3_pin, HIGH, 30000);
    channel_4_value = pulseIn(channel_4_pin, HIGH, 30000);
    channel_5_value = pulseIn(channel_5_pin, HIGH, 30000);
    channel_6_value = pulseIn(channel_6_pin, HIGH, 30000);
    Serial.println("Motor Speed: " + (String)channel_5_value);
  }
  
}


//take remote values and put the into what ever the do
//values are scaled and maped appropritatly
void scale_remote_values()
{
  //set all of the motor speeds to 0
  left_right_motor_speed = 0;
  up_down_motor_speed = 0;
  forward_backward_motor_speed = 0;
  
  //map the uper half of the remote values from 0 to 127 (not as high for motors that never should go full speed)
  if (channel_1_value > 1520){
    left_right_motor_speed = map(channel_1_value, 1520, 1850, 0, 100);
  }
  if (channel_2_value > 1530){
    up_down_motor_speed = map(channel_2_value, 1530, 1800, 0, 100);
  }
  if (channel_5_value > 1600){
    forward_backward_motor_speed = map(channel_5_value, 1600, 1870, 0, 127);
  }
  //map the lower half of the remote values
  if (channel_1_value < 1430){
    left_right_motor_speed = map(channel_1_value, 1430, 1150, 0, -100);
  }
  if (channel_2_value < 1450){
    up_down_motor_speed = map(channel_2_value, 1450, 1160, 0, -100);
  }
  if (channel_5_value < 1400){
    forward_backward_motor_speed = map(channel_5_value, 1400, 1110, 0, -127);
  }
  //constrain the values
  left_right_motor_speed = constrain(left_right_motor_speed, -127, 127);
  up_down_motor_speed = constrain(up_down_motor_speed, -127, 127);
  forward_backward_motor_speed = constrain(forward_backward_motor_speed, -127, 127);

  //set all the servo distances to 0
  LeftRightEyeServoValue = 0;
  UpDownEyeServoValue = 0;
  
  TopEyelidServoValue = 0;
  BottomEyelidServoValue = 0;
  
  //scale the up/down values for the eye servos
  UpDownEyeServoValue = -map(channel_3_value, 1100, 1800, -EYE_SERVO_MOVE_DISTANCE, EYE_SERVO_MOVE_DISTANCE);
  
  //scale the left/right values for the eye servos
  LeftRightEyeServoValue = map(channel_4_value, 1200, 1800, -EYE_SERVO_MOVE_DISTANCE, EYE_SERVO_MOVE_DISTANCE);

  //scale the eyelid servo values
  TopEyelidServoValue = map(channel_3_value, 1100, 1800, -EYELID_SERVO_MOVE_DISTANCE, EYELID_SERVO_MOVE_DISTANCE);
  BottomEyelidServoValue = -TopEyelidServoValue;
}

void read_current_sound()
{
  //map the remote value to sound value
  target_sound = map(channel_6_value, 1100,1800,SILENT,DONT_WATCH);
  //constrain the value
  target_sound = constrain(target_sound,SILENT,DONT_WATCH);
  //find out if the target sound has changed
  if(target_sound != previous_target_sound){
    //set the equal to each other
    previous_target_sound = target_sound;
    //reset the nob time cound
    nob_time = millis();
  }
  //check if the nob has been on the same sound for more that half a second
  if(millis() - 500 > nob_time){
    //set the current sound to that sound
    current_sound = target_sound;
  }
}

//reads the potentiometer and constrains up/down motion
void readPot()
{ 
  //read the potentiometer
  pot_value = analogRead(pot_pin);
  //if the pot value is less than the minimum and it is on a course to go farther         (the faster (negitive) the target speed, the sooner it stops)
  if(pot_value <= (potMin - up_down_motor_speed/2) && up_down_motor_speed < 0){
    //shut off the motor
    up_down_motor_speed = 0;
  }
  //if the pot value is greater than the maximum and it is on a course to go farther        (the faster the target speed, the sooner it stops)
  else if(pot_value >= (potMax - up_down_motor_speed/2) && up_down_motor_speed > 0){
    //shut off the motor
    up_down_motor_speed = 0;
  }
}

//send the data to the motor aruino
void send_data_to_motor_slave()
{
  
  //scale the data to between 0 (full reverse) and 255 (full speed)
  byte left_right = left_right_motor_speed + 127;
  byte up_down = up_down_motor_speed + 127;
  byte forward_backward = forward_backward_motor_speed + 127;
  //start transmitting
  Wire.beginTransmission(MOTOR_ARDUINO);
  //send data
  Wire.write(remote_on);
  Wire.write(left_right);
  Wire.write(up_down);
  Wire.write(forward_backward);
  //end transmission
  Wire.endTransmission(); 
}

//send the data to the sound arduino
void send_data_to_sound_slave()
{
  //start transmitting
  Wire.beginTransmission(SOUND_ARDUINO);
  //send data
  Wire.write((byte)(current_sound));
  //end transmition
  Wire.endTransmission();
}

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600); //connect to the serial moniter
  //declare the channel pins as input
  pinMode(channel_1_pin, INPUT);
  pinMode(channel_2_pin, INPUT);
  pinMode(channel_3_pin, INPUT);
  pinMode(channel_4_pin, INPUT);
  pinMode(channel_5_pin, INPUT);
  pinMode(channel_6_pin, INPUT);
  pinMode(EYE_LED_PIN, OUTPUT);
  //setup the servo shield
  ServoShield.begin();
  ServoShield.setPWMFreq(60);
  //seed the random number generator
  randomSeed(analogRead(A1) * analogRead(A2));
  //set up the neopixel ring
  neoRing.begin();
  neoRing.show();
  yield();
}

//variables for autonomous mode
long int automationTime = 0;
long int autoLastResetTime = 0; //this is set to the current millis when the clock reset. It is subtracted from millis to get automationTime
String autoCurrentAction = "";

//all positions range for -100 to 100
int autoEyelidPos[2] = {0,0};
int autoEyePos[2] = {0,0};
int autoBodyRaisePos = 0;

//because there is no feed back on rotation or track movement, speeds are given instead of positions
//speeds are percentages from -100 to 100
int autoBodyRotationSpeed = 0;
int autoMovementSpeed = 0;

//brightness is a percentage, 0 through 100
int autoEyeBrightness = 50;

//what sound file to play
//enum {SILENT,UP_HERE,NOT_MORON,JUMP_IN_PIT,BOXES_WITH_LEGS,DONT_LOOK,DONT_PANIC,NOT_DEAD,DONT_WATCH,TOTAL_SOUND_NUMBER};
int autoCurrentSound = SILENT;

//positions are give in arrays (ie. [topPos,bottomPos])
//move the eyelids to a certain position over a given amount of time
void MoveEyelids(int startPosT, int startPosB, int endPosT, int endPosB, int startTime, int endTime)
{
  if( automationTime >= startTime && automationTime <= endTime){
    autoEyelidPos[0] = map(automationTime, startTime, endTime, startPosT, endPosT);
    autoEyelidPos[1] = map(automationTime, startTime, endTime, startPosB, endPosB);

    constrain(autoEyelidPos[0], -100, 100);
    constrain(autoEyelidPos[1], -100, 100); 
  }
}

//positions are given in arrays as shown above at MoveEyelids
//move the eye to a certain position over a give amount of time
void MoveEye(int startPosX, int startPosY, int endPosX, int endPosY, int startTime, int endTime)
{
  if(automationTime >= startTime && automationTime <= endTime){
    autoEyePos[0] = map(automationTime, startTime, endTime, startPosX, endPosX);
    autoEyePos[1] = map(automationTime, startTime, endTime, startPosY, endPosY);

    constrain(autoEyePos[0], -100, 100);
    constrain(autoEyePos[1], -100, 100); 
  }
}

//spin the whole body
//speeds between -100 and 100 are given instead of postions (positive speed = clockwise)
void RotateBody(int rotationSpeed, int startTime, int endTime)
{
  if(automationTime >= startTime && automationTime <= endTime){
    autoBodyRotationSpeed = rotationSpeed;
  }
}

//move along the track
//speeds between -100 and 100 are given instead of postions (positive speed = clockwise)
void MoveBody(int movementSpeed, int startTime, int endTime)
{
  if(automationTime >= startTime && automationTime <= endTime){
    autoMovementSpeed = movementSpeed;
  }
}

//move the whole body up/down
void RaiseBody(int startPos, int endPos, int startTime, int endTime)
{
  if(automationTime >= startTime && automationTime <= endTime){
    autoBodyRaisePos = map(automationTime, startTime, endTime, startPos, endPos);
  }
}

//set the eye to a given brightness
void LightEye(int startBrightness, int endBrightness, int startTime, int endTime)
{
  if(automationTime >= startTime && automationTime <= endTime){
    autoEyeBrightness = map(automationTime, startTime, endTime, startBrightness, endBrightness);
  }
}

//set the current sound
void SetSound(int sound, int startTime, int endTime)
{
  if(automationTime >= startTime && automationTime <= endTime){
    autoCurrentSound = sound;
  }
}

void ResetAutomation(int endTime)
{
  if(automationTime >= endTime){
    autoCurrentAction = "";
  }
}

void WriteAutoValues()
{
  //used to map positions
  int value;
  
  //write to eyelids
  value = map(autoEyelidPos[0], -100, 100, TopEyelidServoCenterValue + EYELID_SERVO_MOVE_DISTANCE, TopEyelidServoCenterValue - EYELID_SERVO_MOVE_DISTANCE); //purposfully negated
  write_to_servo(TopEyelidServoPin, value);
  value = map(autoEyelidPos[1], -100, 100, BottomEyelidServoCenterValue - EYELID_SERVO_MOVE_DISTANCE, BottomEyelidServoCenterValue + EYELID_SERVO_MOVE_DISTANCE);
  write_to_servo(BottomEyelidServoPin, value);

  //write to eye
  value = map(autoEyePos[0], -100, 100, UpDownEyeServoCenterValue + EYE_SERVO_MOVE_DISTANCE, UpDownEyeServoCenterValue - EYE_SERVO_MOVE_DISTANCE); //purposefully negated
  write_to_servo(LeftRightEyeServoPin, value);
  value = map(autoEyePos[1], -100, 100, LeftRightEyeServoCenterValue + EYE_SERVO_MOVE_DISTANCE, LeftRightEyeServoCenterValue - EYE_SERVO_MOVE_DISTANCE); //purposefully negated
  write_to_servo(UpDownEyeServoPin, value);

  //take the target position, and calculate a speed
  //write to body raise/lower (the up_down_motor_speed variable is sent to the motor arduino in another function)
  autoBodyRaisePos = -autoBodyRaisePos; //negate so that positve is up
  value = map(autoBodyRaisePos, -100, 100, potMin, potMax);
  int difference = value - pot_value;
  if(abs(difference) > POT_BUFFER){
    //this is negated because it is wired so that positve means look down DON'T NEGATE HERE
    int motor_speed = map(difference, -up_down_full_speed_distance, up_down_full_speed_distance,-127,127);
    up_down_motor_speed = constrain(motor_speed, -127, 127);
  }
  else{
    up_down_motor_speed = 0;
  }

  //write to body rotaiton
  left_right_motor_speed = map(autoBodyRotationSpeed,-100,100,-127,127);
  left_right_motor_speed = constrain(left_right_motor_speed,-127,127); //this should never happen, but just in case

  //write to body movement
  forward_backward_motor_speed = map(autoMovementSpeed,-100,100,-127,127);
  forward_backward_motor_speed = constrain(forward_backward_motor_speed,-127,127); //this should never happen, but just in case

  //write to eye light
  value = map(autoEyeBrightness, 0, 100, 0, 255);
  write_to_neo_ring(value);

  //play the sound
  current_sound = autoCurrentSound;
}

//this should be called each loop to reset rotation and movement speeds
void ResetSpeeds()
{
  autoBodyRotationSpeed = 0;
  autoMovementSpeed = 0;
  autoCurrentSound = SILENT;
}

//called when no auto sequence is happening to center and reset everything
void AutoDoingNothing()
{
  //reset speeds is already called so we don't need to worry about that
  autoEyeBrightness = 50;
  autoEyePos[0] = 0;
  autoEyePos[1] = 0;
  autoEyelidPos[0] = 0;
  autoEyelidPos[1] = 0;
  autoBodyRaisePos = 0;
}

void TestAutomation()
{
  SetSound(NOT_MORON,0,3000);
  LightEye(0,100,0,30);

  MoveEye(0,0,75,-75,1000,2000);
  MoveEye(75,-75,0,0,3000,3500);
  
  RaiseBody(-100,100,0,1000);
  RaiseBody(100,0,3000,3100);
  RaiseBody(0,-100,4000,4100);
  
  LightEye(100,0,2000,3000);

  RotateBody(100,500,1000);
  RotateBody(-100,2000,2500);

  MoveBody(100,3000,3500);
  
  ResetAutomation(5000);
}

void IAmNotAMoronAutomation()
{
  SetSound(NOT_MORON,0,3000);
  
  RaiseBody(0,100,0,500); //rase head and eye up and open eye
  MoveEyelids(0,0,100,100,0,500);
  MoveEye(0,0,0,100,0,500);

  RaiseBody(100,0,500,1500); //lower head and shake eye
  MoveEye(0,100,100,0,500,750);
  MoveEye(100,0,-100,0,750,1000);
  MoveEye(-100,0,0,0,1000,1250);

  MoveEyelids(100,100,-100,-100,750,1000); //blink eye

  RaiseBody(0,-40,1500,2000); //look down and open eye
  MoveEyelids(0,0,100,100,1500,2000);

  MoveEye(0,0,100,0,1750,2000); //shake eye again
  MoveEye(100,0,-100,0,2000,2500);
  MoveEye(-100,0,0,0,2500,2750);

  
  
  ResetAutomation(10000); //this extra delay is only temporary, it sould be 3000
}

void UpHereAutomation()
{
  SetSound(UP_HERE,0,3000); //start sound
  RaiseBody(0,100,0,500); //raise head
  MoveEyelids(0,0,-100,-100,0,500); //close eye

  RaiseBody(100,-100,1000,2000); //lower head
  MoveEyelids(-100,-100,100,100,1000,1500); //open eye

  RaiseBody(-100,0,1500,2000); //raise the head
  MoveEye(0,0,0,100,1500,1750); //raise eye
  MoveEye(0,100,0,0,1750,2000); //lower eye

  ResetAutomation(10000); //this extra delay is only temporary, it sould be 3000
}

void JumpAuto()
{
  SetSound(JUMP_IN_PIT,0,13000); //start sound
  ResetAutomation(14000);
}
void BoxesAuto()
{
  SetSound(BOXES_WITH_LEGS,0,11000); //start sound
  ResetAutomation(12000);
}
void DontLookAuto()
{
  SetSound(DONT_LOOK,0,14000); //start sound

  //shake the head "Not interested in anything else, don't touch anything else, don't even look at anything else."
  MoveEyelids(0,0,-100,-100,0,500); //close eye
  

  
  ResetAutomation(15000);
}
void DontPanicAuto()
{
  SetSound(DONT_PANIC,0,27000); //start sound
  ResetAutomation(28000);
}
void NotDeadAuto()
{
  SetSound(NOT_DEAD,0,5000); //start sound
  ResetAutomation(10000);
}
void DontWatchAuto()
{
  SetSound(DONT_WATCH,0,8000); //start sound
  ResetAutomation(10000);
}

void ActAuto()
{
  //before this is called, random numbers are generated for everything

  MoveBody(100,1000,9000);
  
  RaiseBody(100,-100,1000,2000); //lower head
  MoveEyelids(-100,-100,100,100,1000,1500); //open eye

  RaiseBody(-100,0,1500,2000); //raise the head
  MoveEye(0,0,0,100,1500,1750); //raise eye


  ResetAutomation(10000);
}


//chooses random things to do, then runs the function that lists the actions
void Automate()
{
  automationTime = millis() - autoLastResetTime;

  if(autoCurrentAction == ""){
    //when doing nothing, reset everything
    AutoDoingNothing();
    int randomNumber = 0;//random(0,1000);

    //testing section
    /*
    if(true){
      //testing only
      autoCurrentAction = "Not Moron";
      autoLastResetTime = millis();
    }
    */

    //if action should be done
    if(randomNumber <= 10)
    {
      //reset 
      autoLastResetTime = millis(); //THIS MUST BE CALLED WHENEVER AN AUTO STARTS!
      if(randomNumber == 0)
      {
        autoCurrentAction = "Not Moron";
      }
      else if(randomNumber == 1)
      {
        autoCurrentAction = "Up Here";
      }
      else if(randomNumber == 2)
      {
        autoCurrentAction = "Jump";
      }
      else if(randomNumber == 3)
      {
        autoCurrentAction = "Boxes";
      }
      else if(randomNumber == 4)
      {
        autoCurrentAction = "Dont Look";
      }
      else if(randomNumber == 5)
      {
        autoCurrentAction = "Dont Panic";
      }
      else if(randomNumber == 6)
      {
        autoCurrentAction = "Not Dead";
      }
      else if(randomNumber == 7)
      {
        autoCurrentAction = "Dont Watch";
      }
      else if(randomNumber >= 8 && randomNumber <= 10)
      {
        //set random values for act auto
        autoCurrentAction = "Act";
      }
    }
  }

  //this needs to be called every auto loop
  ResetSpeeds();
  
  //if not doing something
  //probability of choosing random thing
  //save that thing and run its code every frame
  //reset the automation timer

  if(autoCurrentAction == "Test"){ TestAutomation(); }
  else if(autoCurrentAction == "Not Moron"){ IAmNotAMoronAutomation(); }
  else if(autoCurrentAction == "Up Here"){ UpHereAutomation(); }
  else if(autoCurrentAction == "Jump"){ JumpAuto(); }
  else if(autoCurrentAction == "Boxes") { BoxesAuto(); }
  else if(autoCurrentAction == "Dont Look"){ DontLookAuto(); }
  else if(autoCurrentAction == "Dont Panic"){ DontPanicAuto(); }
  else if(autoCurrentAction == "Not Dead"){ NotDeadAuto(); }
  else if(autoCurrentAction == "Dont Watch"){ DontWatchAuto(); }
  else if(autoCurrentAction == "Act"){ ActAuto(); }

  Serial.println(autoCurrentAction);
  
  //write values to the motors
  WriteAutoValues();
}

void loop() {

  //debug
  Serial.println("working");

  //get input from the remote
  update_channels();
  //if the remote is on, scale the values to the motor speeds and read the current sound
  if(remote_on){
    scale_remote_values();
    read_current_sound();

    //write to the servo shield
    update_eye_servos();
    
    //light up the neopixel ring
    write_to_neo_ring(125);
  }
  
  //if the remote is off, automate
  else{
    Automate();
  }
  
  //constrain the up/down motion (pot value is also updated here)
  readPot();
  
  //set the data over I2C
  send_data_to_motor_slave();
  send_data_to_sound_slave();
}
