/*
 * Wheatley Sound Slave
 * Rio King
 * 
 * This code listens for data over I2C and drives
 * the Adafruit sound board based on that data.
 */

#include <Wire.h>
//set the port to connect to
const int I2C_Port = 2;

//define numbers for the different sounds (the same numbers are defined in the master code)
enum {SILENT,UP_HERE,NOT_MORON,JUMP_IN_PIT,BOXES_WITH_LEGS,DONT_LOOK,DONT_PANIC,NOT_DEAD,DONT_WATCH,TOTAL_SOUND_NUMBER};
//the sound to play
int current_sound = UP_HERE;
int previous_sound = UP_HERE;

void setup() {
  Serial.begin(9600);
  //connect to I2C
  Wire.begin(I2C_Port);
  //the function to run when data is received
  Wire.onReceive(receive_data);
  // set all the sound pins to output and write them high
  for(int i = UP_HERE; i < TOTAL_SOUND_NUMBER; ++i){
    pinMode(i + 1,OUTPUT);
    digitalWrite(i + 1,HIGH);
  }
}

//run when data is recieved
void receive_data(int howMany) {
  //read the number that is being sent
  current_sound = Wire.read();
}

void loop() {
  // put your main code here, to run repeatedly:
  //check if a new sound needs to play
  Serial.println(current_sound);
  if(current_sound != previous_sound){
    //release the previous pin
    digitalWrite(previous_sound + 1, HIGH);
    //check if the sound is silent
    if(current_sound != SILENT){
      //ground the new pin
      digitalWrite(current_sound + 1, LOW);
    }
    //set previous sound equal to current sound
    previous_sound = current_sound;
  }
}
