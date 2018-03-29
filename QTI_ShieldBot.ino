#include <DRV8835MotorShield.h>

#define MAX_SPEED_L 250.0
#define MAX_SPEED_R 200.0

DRV8835MotorShield motors;

int m1CurSpeed;
int m2CurSpeed;
int vL, vR;

// Perform these steps with the Arduino is first powered on
void setup()
{
  Serial.begin(9600); 
  pinMode(13, OUTPUT);
  m1CurSpeed = 0;
  m2CurSpeed = 0;
  
  // Uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipM1(true);
  motors.flipM2(true);

  while(digitalRead(3) == 0)
  {}
}

// This code repeats indefinitely
void loop()
{
  int pins = PIND;                           // Get values of pins D0-D7
  
  DDRD |= B11110000;                         // Set direction of Arduino pins D4-D7 as OUTPUT
  PORTD |= B11110000;                        // Set level of Arduino pins D4-D7 to HIGH
  delayMicroseconds(230);                    // Short delay to allow capacitor charge in QTI module
  DDRD &= B00001111;                         // Set direction of pins D4-D7 as INPUT
  PORTD &= B00001111;                        // Set level of pins D4-D7 to LOW
  delayMicroseconds(230);                    // Short delay
  pins >>= 4;                                // Drop off first four bits of the port; keep only pins D4-D7
  
  // Determine how to steer based on state of the four QTI sensors
  int turn7 = 100;
  int slightTurn = 70;
  int medTurn = 50;
  int hardTurn = 5;
  byte binPins = (byte) pins;
 
  //pins = pins ^ B1111; //1=black, 0=white
  
  //String pString = (String) binPins;
  //Serial.println(binPins);
  //Serial.println(pString);
  Serial.println(pins, BIN);

  switch(pins)                               // Compare pins to known line following states
  {
    case B1000:  //hard left                      
      vL = hardTurn;                            // -100 to 100 indicate course correction values
      vR = 150;                              // -100: full reverse; 0=stopped; 100=full forward
      break;
    case B1100:  //med left                      
      vL = medTurn;
      vR = turn7;
      break;
    case B1110:  //med left                      
      vL = medTurn;
      vR = turn7;
      break;
    case B0100:  //slight left                      
      vL = slightTurn;
      vR = 100;
      break;
    case B0110:    //straight                   
      vL = 100;
      vR = 100;
      break;
    case B0001:      //hard right                  
      vL = 150;
      vR = hardTurn;
      break;
    case B0011:  //med right                      
      vL = turn7;
      vR = medTurn;
      break;
    case B0111:      //med right                  
      vL = turn7;
      vR = medTurn;
      break;
    case B0010:        //slight right                
      vL = 100;
      vR = slightTurn;
      break;
    case B0000://kill
      vL = 0;
      vR = 0;
      break;
    case B1111://kill
      vL = 0;
      vR = 0;
      break;
  }

  //set new target speed to reach
  controlSpeed((int)(((double)(MAX_SPEED_L * vL)/100.0)), (int)(((double)(MAX_SPEED_R * vR)/100.0)), loopStartTime);
}

void controlSpeed(int m1TargetSpeed, int m2TargetSpeed, unsigned long stTime){
  // max acceleration possible (as a percentage (0-100) of max speed per millisecond)
  double accelFactor = 100;
  double decelFactor = 100;
  //Serial.println("m1 Target: " + (String)m1TargetSpeed);
  //----------------motor 1 control speed
  if(m1CurSpeed < m1TargetSpeed){  //accelerate
    m1CurSpeed = min(m1CurSpeed + accelFactor * MAX_SPEED_L * (stTime - millis()), m1TargetSpeed);
  }else if(m1CurSpeed > m1TargetSpeed){ //decelerate
    m1CurSpeed = max(m1CurSpeed - decelFactor * MAX_SPEED_L * (stTime - millis()), m1TargetSpeed);
  }
  //Serial.println("m1 cur: " + (String)m1CurSpeed);
  //----------------motor 2 control speed
  if(m2CurSpeed < m2TargetSpeed){  //accelerate
    m2CurSpeed = min(m2CurSpeed + accelFactor * MAX_SPEED_R * (stTime - millis()), m2TargetSpeed);
  }else if(m2CurSpeed > m2TargetSpeed){ //decelerate
    m2CurSpeed = max(m2CurSpeed - decelFactor * MAX_SPEED_R * (stTime - millis()), m2TargetSpeed);
  }
  //set new speeds
  setMotorSpeeds();
}

void setMotorSpeeds(){
  //error checking speeds
  if(m1CurSpeed > MAX_SPEED_L){
    m1CurSpeed = MAX_SPEED_L;
  }else if(m1CurSpeed < -MAX_SPEED_L){
    m1CurSpeed = -MAX_SPEED_L;
  }
  if(m2CurSpeed > MAX_SPEED_R){
    m2CurSpeed = MAX_SPEED_R;
  }else if(m2CurSpeed < -MAX_SPEED_R){
    m2CurSpeed = -MAX_SPEED_R;
  }
  //set motors
  //Serial.println("m1 write: " + (String)m1CurSpeed);
  motors.setM2Speed(m2CurSpeed);
  motors.setM1Speed(m1CurSpeed);
}

