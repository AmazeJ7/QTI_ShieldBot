#include <DRV8835MotorShield.h>

#define MAX_SPEED_L 142.0 //135 //142
#define MAX_SPEED_R 130.0 //120 //130
#define loopUpdate 9
DRV8835MotorShield motors;

int m1CurSpeed;
int m2CurSpeed;
int vL, vR;

unsigned long hardTurnCooldown = 1000;
unsigned long hardTurnTimer;

int prevPins = 0;
int loopCount;
int pinTotal[loopUpdate];
int highestIndex=0;
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
  pinTotal[0] = 0;
}

// This code repeats indefinitely
void loop()
{
  unsigned long loopStartTime = millis();
                           // Get values of pins D0-D7
  
  DDRD |= B11110000;                         // Set direction of Arduino pins D4-D7 as OUTPUT
  PORTD |= B11110000;                        // Set level of Arduino pins D4-D7 to HIGH
  delayMicroseconds(230); //230                   // Short delay to allow capacitor charge in QTI module
  DDRD &= B00001111;                         // Set direction of pins D4-D7 as INPUT
  PORTD &= B00001111;                        // Set level of pins D4-D7 to LOW
  delayMicroseconds(230);                    // Short delay
  int pins = PIND;  
  pins >>= 4;                                // Drop off first four bits of the port; keep only pins D4-D7
  //pins = pins ^ B1111; //1=black, 0=white
  //byte binPins = (byte) pins;
  /*
  if(loopCount < loopUpdate){
    pinTotal[loopCount] = pins;
  }else{
    loopCount = 0;
    int majority[loopUpdate];
    //majority[0] = 1; means 1 instance of whatever in in pinTotal[0]
    highestIndex = 0;
    for( int i=0; i < loopUpdate; i++){
      boolean found = false;
      for(int j=0; j < i; j++){
        if(pinTotal[j] == pinTotal[i]){//matching with first/original majority[j]
          found = true;
          majority[j] +=1;
          if(majority[highestIndex] < majority[j]){
            highestIndex = j;
          }
          j = i;
        }
      }
      if(!found){
        majority[i] = 1;
      }
    }
    Serial.print("Average: ");
    Serial.println(pins, BIN);
  }
  pins = pinTotal[highestIndex];
  */
  
  if(pins == B0001 || pins == B1000){
    hardTurnTimer = loopStartTime + hardTurnCooldown;
  }
  int correctionPins = prevPins;
  if(pins == B0000 && prevPins == B0001){
    pins = B0001;
    /*
    if(loopStartTime > hardTurnTimer){
      pins = B0011;
    }*/
  }
  else if(pins == B0000 && prevPins == B1000){
    pins = B1000;
    /*
    if(loopStartTime > hardTurnTimer){
      pins = B1100;
    }*/
  }
  else{
    prevPins = pins;
  }
  
  
  // Determine how to steer based on state of the four QTI sensors
  int turn7 = 100;
  int slightTurn = 60; //70      //90 for line refinding code version
  int medTurn = 30; //30         //70 
  int hardTurn = 0; //15        //60
  //60, 30, 0 works
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
  //controlSpeed((int)MAX_SPEED_L, (int)(((double)(MAX_SPEED_R * 70)/100.0)), loopStartTime);
  //loopCount++;
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
  delay(10); 
  motors.setM1Speed(m1CurSpeed);
}

