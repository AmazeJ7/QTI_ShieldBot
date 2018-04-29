#include <DRV8835MotorShield.h>
//Left:  152
//Right: 160
#define MAX_SPEED_L 147.0 //142
#define MAX_SPEED_R 135.0 //130
#define loopUpdate 5
#define ErrorLogAmt 200

DRV8835MotorShield motors;
int m1CurSpeed;
int m2CurSpeed;
long vL, vR;
int errorLogL[ErrorLogAmt];
int errorLogR[ErrorLogAmt];
int errorLogIndex = 0;
long accumulatorL = 50 * 2000;
long accumulatorR = 50 * 2000;
int K_P = 100; //2^8
int K_I = 5; //2^2 (and *30 for accum)
int K_D = 0; //2^0
int divFactor = 110;

unsigned long overallCounter = 0;
unsigned long hardTurnCooldown = 1000;
unsigned long hardTurnTimer;

unsigned long killTimer;

int allBlack = 0;
int prevPins = 0;

int leftSpeeds[] =  {56, 65, 75, 83, 93,  104, 114, 124, 134, 142, 152};
int rightSpeeds[] = {60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160};

// Perform these steps with the Arduino is first powered on
void setup()
{
  //pinMode(10, OUTPUT);
  //pinMode(9, OUTPUT);
  Serial.begin(9600); 
  //pinMode(13, OUTPUT);
  m1CurSpeed = 0;
  m2CurSpeed = 0;
  
  // Uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipM1(true);
  motors.flipM2(true);
  killTimer = 0;
  Serial.println("test");
  while(digitalRead(11) == 0)
  {}
  Serial.println("test2");
  
}

// This code repeats indefinitely
void loop()
{
  unsigned long loopStartTime = millis();
  // Averaging:
  int pinTotal[loopUpdate];
  int highestIndex = 0;
  int pins;

  if(allBlack == 0){//no flag set, continue killTimer = current time
    killTimer = millis();
  }
  
  /*for(int i = 0; i < loopUpdate; i++){
    pinTotal[i] = readPins(330);
    //Serial.println("pinRead#: " + (String) i + ": " + (String) pinTotal[i]);
    delay(1);
  }
  int majority[loopUpdate];
  //majority[0] = 1; means 1 instance of whatever in in pinTotal[0]
  highestIndex = 0;
  for(int i = 0; i < loopUpdate; i++){
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
  pins = pinTotal[highestIndex];*/
  pins = readPins(230);
  //pins = B1000;
  //Serial.print("Average: ");
  //RCtime(4);
  //Serial.println(pins, BIN);
  //------------------------------------------------PID Controller -------------------------------------------
  error(pins, loopStartTime);
  int nxtIndex = (errorLogIndex + 1) % ErrorLogAmt;// find circular next index
  //Serial.print("nxtI: " + (String)nxtIndex + " ");
  //update sum of array
  /*accumulatorL += vL;
  accumulatorR += vR;
  accumulatorL -= errorLogL[errorLogIndex];
  accumulatorR -= errorLogR[errorLogIndex];*/
  if(accumulatorL > 10 * 2000){
    accumulatorL = 10 * 2000;
  }else if(accumulatorL < -10 * 2000){
    accumulatorL = -10 * 2000;
  }
  if(accumulatorR > 10 * 2000){
    accumulatorR = 10 * 2000;
  } else if(accumulatorR < -10 * 2000){
    accumulatorR = -10 * 2000;
  }
  accumulatorL += (vL - vR);
  accumulatorR += (vR - vL);
  //change current error on array
  errorLogL[errorLogIndex] = vL;
  errorLogR[errorLogIndex] = vR;
  //PID function
  long PID_L = vL * K_P;
  long PID_R = vR * K_P;
  //Serial.print("PID_L P: " + (String)PID_L + " ");
  //PID_L += K_I * ((double)accumulatorL / ErrorLogAmt); //sum of all past 100
  //PID_R += K_I * ((double)accumulatorR / ErrorLogAmt);
  PID_L += K_I * ((double)accumulatorL / 2000); //sum of all past 100
  PID_R += K_I * ((double)accumulatorR / 2000);
  //Serial.print("PID_L PI: " + (String)PID_L + " ");
  PID_L += K_D * (errorLogL[errorLogIndex] - errorLogL[nxtIndex]); //difference from now compared to 100 reads ago
  PID_R += K_D * (errorLogR[errorLogIndex] - errorLogR[nxtIndex]);
  //Serial.print("PID_L PID: " + (String)PID_L + " ");
  //update velocities of each wheel
  vL = (double)PID_L / divFactor;
  vR = (double)PID_R / divFactor;
  //Serial.println(" vL: " + (String)vL + "  vR: " + (String)vR + "  AvgL: " + (String)((double) accumulatorL / ErrorLogAmt));
  //set new target speed to reach
  //controlSpeed((int)(((double)(MAX_SPEED_L * vL)/100.0)), (int)(((double)(MAX_SPEED_R * vR)/100.0)), loopStartTime);

  motors.setM1Speed((int)(((double)(MAX_SPEED_L * vL)/100.0)));
  //delay(2);
  motors.setM2Speed((int)(((double)(MAX_SPEED_R * vR)/100.0)));
  
  //Serial.println((int)(((double)(MAX_SPEED_L * (double)100.0)/100.0)));
  //controlSpeed((int)(((double)(MAX_SPEED_L * (double)100.0)/100.0)), (int)(((double)(MAX_SPEED_R * (double) 100.0)/(double)100.0)), loopStartTime);
  /*if(killTimer + 1000 > millis()){
    controlSpeed((int)(((double)(MAX_SPEED_L * (double)100.0)/100.0)), (int)(((double)(MAX_SPEED_R * (double) 100.0)/(double)100.0)), loopStartTime);
  }else if(killTimer + 3000 > millis()){
    controlSpeed((int)(((double)(MAX_SPEED_L * (double)68.0)/100.0)), (int)(((double)(MAX_SPEED_R * (double) 100.0)/(double)100.0)), loopStartTime);
  }else{
    controlSpeed((int)(((double)(MAX_SPEED_L * (double)0)/100.0)), (int)(((double)(MAX_SPEED_R * (double) 0)/(double)100.0)), loopStartTime);
  }*/
  overallCounter++;
  errorLogIndex = nxtIndex;
}

//READ PIN grayscale WIP
long RCtime(int sensPin){
  int arduinoPins[] =  {2, 3, 4, 5};
  long result[] = {0, 0, 0, 0};
  for(int i = 0; i< sizeof(arduinoPins); i++){
    pinMode(arduinoPins[i], OUTPUT);       // make pin OUTPUT
    digitalWrite(arduinoPins[i], HIGH);    // make pin HIGH to discharge capacitor - study the schematic
    delay(1);                       // wait a  ms to make sure cap is discharged
    
    pinMode(arduinoPins[i], INPUT);        // turn pin into an input and time till pin goes low
    digitalWrite(arduinoPins[i], LOW);     // turn pullups off - or it won't work
    while(digitalRead(arduinoPins[i])){    // wait for pin to go low
      result[i]++;
    }/*
    if(){
      result[i]
    }*/
  }    
                
  return result;
}


//READ PINS-------------------------------------------------------------------------------------------------------------------------
int readPins(int usDelay){
  // Get values of pins D0-D7
  DDRD |= B00111100;                         // Set direction of Arduino pins D4-D7 as OUTPUT
  PORTD |= B00111100;                        // Set level of Arduino pins D4-D7 to HIGH
  delayMicroseconds(usDelay); //230                   // Short delay to allow capacitor charge in QTI module
  DDRD &= B11000011;                         // Set direction of pins D4-D7 as INPUT
  PORTD &= B11000011;                        // Set level of pins D4-D7 to LOW
  delayMicroseconds(usDelay);                    // Short delay
  int pins = PIND;  
  pins >>= 2;                                // Drop off first four bits of the port; keep only pins D4-D7
  pins = pins % 16;
  //pins = pins ^ B1111; //1=black, 0=white
  //byte binPins = (byte) pins;
  return pins;
}

//returns vL and vR as an int[2] based on previous case style----------------------------------------------------------------------
void error(int pins, unsigned long loopStartTime){
  int slightTurnL = 60; //60  
  int slightTurnR = 60; //60     
  int medTurnL = 30; //30   
  int medTurnR = 30; //30       
  int hardTurnL = 0; //0   
  int hardTurnR = 0; //0      
  //60, 30, 0 for prototype version
  
  /*if(pins == B0001 || pins == B1000){
    hardTurnTimer = loopStartTime + hardTurnCooldown;
  }*/
  //fell off after hard right
  //if(pins == B0000 && prevPins == B0001){
  if(pins == B0000){
    pins = prevPins;
    /*
    if(loopStartTime > hardTurnTimer){
      pins = B0011;
    }*/
  }
  //fell off after hard left
  else if(pins == B0000 && prevPins == B1000){
    pins = B1000;
    /*
    if(loopStartTime > hardTurnTimer){
      pins = B1100;
    }*/
  }/*
  else if(pins == B1111 && prevPins != B1111){//first occurance of all black
    allBlack = 1; //set flag
  }
  //false alarm of all black, didn't just stop too late and skip over
  else if(pins != B1111 && prevPins == B1111 && killTimer + 20 < millis()){
    allBlack = 0; //clear flag
  }*/
  else{
    prevPins = pins;
  }
  //20 ms+ of seeing black needed
  /*if(killTimer + 20 < millis()){
    pins = B1111;
  }*/
  // Determine how to steer based on state of the four QTI sensors
  
  switch(pins)                               // Compare pins to known line following states
  {
    case B1000:  //hard left                      
      vL = hardTurnL;                            // -100 to 100 indicate course correction values
      vR = 150;                              // -100: full reverse; 0=stopped; 100=full forward
      break;
    case B1100:  //med left                      
      vL = medTurnL;
      vR = 100;
      break;
    case B1110:  //med left                      
      vL = medTurnL;
      vR = 100;
      break;
    case B0100:  //slight left                      
      vL = slightTurnL;
      vR = 100;
      break;
    case B0110:    //straight                   
      vL = 115;
      vR = 115;
      break;
    case B0001:      //hard right                  
      vL = 150;
      vR = hardTurnR;
      break;
    case B0011:  //med right                      
      vL = 100;
      vR = medTurnR;
      break;
    case B0111:      //med right                  
      vL = 100;
      vR = medTurnR;
      break;
    case B0010:        //slight right                
      vL = 100;
      vR = slightTurnR;
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
  motors.setM1Speed(m1CurSpeed);
  //delay(2);
  motors.setM2Speed(m2CurSpeed);
  //analogWrite(10, 220 + ((double)(m1CurSpeed / MAX_SPEED_L) * 35));
  //analogWrite(9, 220 + ((double)(m2CurSpeed / MAX_SPEED_R) * 35));
}

