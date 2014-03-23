/*
Carlo Cassinari - cashbit
2014-03-23
Sketch to drive a robot with hub-ee wheels and Arduino.

https://github.com/cashbit/hub-ee-arduino
 
To drive send these commands:
 
  eme:0 --> emergency stop

  dir:1 --> set direction, 1 forward, 0 backward
  
  dis:100 --> set distance, motor will go in the setup direction for 100 steps
  
  vel:0 --> set speed, 0 to stop, 255 for max speed
  
  rot:90 --> rotate clockwise 90 steps, with -90 rotate counterclockwise 90 steps
 

The robot sends always his status with a json string like this: 

{"sx":{"speed":100,"counter":23,"direction":1},"dx":{"speed":100,"counter":23,"direction":1},"distanceleft":103.4}

When distanceleft equals 0, the robot has runned for the requested steps with dis command.
 
Examples:
  
  make 100 steps forward, max speed:  dir:1,dis:100,vel:255
  
  rotate by 90 steps on the left at speed 100: rot:90;vel:100
  
  
Note: every time the robot is approaching the distanceleft 0 value, it sets automatically the speed to 50 to slow down (only if the speed is more than 50).

*/

#include <HUBeeBMDWheel.h>

// wheels setup
HUBeeBMDWheel motor1Wheel;
HUBeeBMDWheel motor2Wheel;

int loopFrequency = 10 ; // 2 Hertz
int loopDelay = 100 ;

int motor1DirectionForward = 1 ;
int motor2DirectionForward = 0 ;
int motor1DirectionReverse = 0 ;
int motor2DirectionReverse = 1 ;

int motor1CounterSign = -1 ;
int motor2CounterSign = +1 ;


int motor1Speed = 0, motor2Speed = 0;

int motor1QeiAPin  = 3; //external interrupt 1 (UNO) or 0 (Leonardo)
int motor1QeiBPin  = 7;
int motor2QeiAPin = 2; //external interrupt 0 (UNO) or 1 (Leonardo)
int motor2QeiBPin = 4;

volatile int motor1QeiCounts = 0, motor2QeiCounts = 0;
volatile int motor1Counter = 0, motor2Counter = 0;

int motor1Direction = motor1DirectionForward ;
int motor2Direction = motor2DirectionForward ;

// command setup

int speedSetPoint = 0;
float distanceToCover = 0 ;
float coveredDistance = 0 ;
float distanceToDestination = 0 ;
int forwardAcceleration = 20 ; // acceleration 
int forwardDeceleration = 10 ;
int reverseDeceleration = 10 ;
int reverseAcceleration = 10 ;
int approachingSpeed = 50 ;

int robotRotating = 0 ; // -1 counterclockwise, 0 stopped, 1 counterclockwise
int robotDirection = 1 ; // -1 reverse, 0 stopped, 1 forward
float robotSpeed = 0 ; // 0 - 255 

// Communications
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

String _oldbuffer ; 
  
void setup()
{
  pinMode(motor1QeiAPin, INPUT_PULLUP);
  pinMode(motor2QeiAPin, INPUT_PULLUP);
  pinMode(motor1QeiBPin, INPUT_PULLUP);
  pinMode(motor2QeiBPin, INPUT_PULLUP);
  motor1Wheel.setupPins(8,11,9); 
  motor2Wheel.setupPins(12,13,10);
  
  attachInterrupt(1, Motor1quickQEI, CHANGE);
  attachInterrupt(0, Motor2quickQEI, CHANGE);
  
  _oldbuffer = "" ;
  
  Serial.begin(9600);
  //inputString.reserve(200);
  //inputString = "dir:1,dis:2000,vel:150" ;
  //stringComplete = true ;
  
  loopDelay = (int)round(1000/loopFrequency);
  
}


void loop()
{
 
  if (stringComplete) {
      stringComplete = false ;
      processMessage(inputString);
      inputString = "" ;
  }
  
  coveredDistance = (abs((float)motor1Counter) + abs((float)motor2Counter)) / 2 ;
  distanceToDestination = distanceToCover - coveredDistance ;
  
  if (distanceToDestination <= 0){
    distanceToDestination = 0 ;
    emergency(0) ;
  } else {
    /*
    float distanceDeceleration = distanceToDestination/robotSpeed ;
    if (distanceDeceleration < 0.4){
      // we start to decelerate to avoid fast stopping
      if (speedSetPoint > approachingSpeed) setSpeedSetPoint(approachingSpeed);
    }
    */
    int deltaSpeed = calcDeltaSpeed() ;
    motor1Speed += deltaSpeed ;
    if (motor1Speed > 255) motor1Speed = 255 ;
    if (motor1Speed < 0) motor1Speed = 0 ;
    motor2Speed += deltaSpeed ;
    if (motor2Speed > 255) motor2Speed = 255 ;
    if (motor2Speed < 0) motor2Speed = 0 ;
  }

  motor1Wheel.setDirectionMode(motor1Direction);
  motor2Wheel.setDirectionMode(motor2Direction);
  motor1Wheel.setMotorPower(motor1Speed);
  motor2Wheel.setMotorPower(motor2Speed);
  

  getStatus(0) ;
  
  delay(loopDelay);

}

void Motor1quickQEI()
{
  //a fast(ish) QEI function
  int state = 0;
  state = digitalRead(motor1QeiAPin) << 1;
  state = state|digitalRead(motor1QeiBPin);
  switch (state)
  {
    case 0:
      motor1QeiCounts--;
      break;
    case 1:
      motor1QeiCounts++;
      break;
    case 2:
      motor1QeiCounts++;
      break;
    case 3:
      motor1QeiCounts--;
      break;
  }
  motor1Counter = motor1QeiCounts * motor1CounterSign ;
}

void Motor2quickQEI()
{
  //a fast(ish) QEI function
  int state = 0;
  state = digitalRead(motor2QeiAPin) << 1;
  state = state|digitalRead(motor2QeiBPin);
  switch (state)
  {
    case 0:
      motor2QeiCounts--;
      break;
    case 1:
      motor2QeiCounts++;
      break;
    case 2:
      motor2QeiCounts++;
      break;
    case 3:
      motor2QeiCounts--;
      break;
  }
  motor2Counter = motor2QeiCounts * motor2CounterSign ;
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}


String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void processMessage(String data){
  int index = 0 ;
  String command = getValue(data,',',index) ;
  while (!command.equals("")){
    processCommand(command);
    index++;
    command = getValue(data,',',index) ;  
  }
}

void processCommand(String command){
  //
  String instruction = getValue(command,':',0);
  int value = getValue(command,':',1).toInt();
  int error = 1 ;
  //Serial.print("Processing command: ");
  //Serial.print(instruction);
  //Serial.print(":");
  //Serial.println(value);
  if (instruction == "eme") error = emergency(0) ;
  if (instruction == "vel") error = setSpeedSetPoint(value);
  if (instruction == "dir") error = setMotorDirection(value);
  if (instruction == "dis") error = setDistanceToCover(value);
  if (instruction == "rot") error = setRotationToValue(value);
  if (instruction == "sta") error = getStatus(1) ;
  if (error) {
    Serial.print("{\"error\" : \"Error processing command: ");
    Serial.print(command);
    Serial.println("\"}");
  }
}

int getStatus(int value){
  char _buffer[300];
  char float_str3[16] = "";
  dtostrf(distanceToDestination,4,2,float_str3);  
  sprintf(_buffer,  "{\"sx\":{\"speed\":%d,\"counter\":%d,\"direction\":%d},\"dx\":{\"speed\":%d,\"counter\":%d,\"direction\":%d},\"distanceleft\":%s}",
    motor2Speed,motor2Counter,motor2Direction == motor2DirectionForward,
    motor1Speed,motor1Counter,motor1Direction == motor1DirectionForward,
    float_str3
  );
  
  String _bufferString = _buffer ;
  if ((_bufferString != _oldbuffer) || value){
    Serial.println(_buffer);
    _oldbuffer = _bufferString ;
  }
  return 0 ;
}
int emergency(int value){
  robotDirection = 0;
  robotRotating = 0 ;
  motor1Speed = 0 ;
  motor2Speed = 0 ;
  motor1Wheel.setMotorPower(motor1Speed);
  motor2Wheel.setMotorPower(motor2Speed);
  distanceToCover = 0 ;
  return 0 ;
}

int setSpeedSetPoint(int value){
  speedSetPoint = value ;
  return 0 ;
}

int setMotorDirection(int value){
  if (value){
    robotDirection = 1 ;
    motor1Direction = motor1DirectionForward ;
    motor2Direction = motor2DirectionForward ;
  } else {
    robotDirection = -1 ;
    motor1Direction = motor1DirectionReverse ;
    motor2Direction = motor2DirectionReverse ;
  } 
  return 0 ;
}

int setDistanceToCover(int value){
  robotRotating = 0 ;
  motor1QeiCounts = 0 ;
  motor2QeiCounts = 0 ;
  motor1Counter = motor1QeiCounts * motor1CounterSign ;
  motor2Counter = motor2QeiCounts * motor2CounterSign ;
  distanceToCover = abs((float)value) ;
  return 0 ;
}

int setRotationToValue(int value){
  robotDirection = 0 ;
  if (value > 0){
    robotRotating = 1 ;
    motor1Direction = motor1DirectionForward ;
    motor2Direction = motor2DirectionReverse ;
    setDistanceToCover(abs(value)) ;
  } else if (value < 0) {
    robotRotating = -1 ;
    motor1Direction = motor1DirectionReverse ;
    motor2Direction = motor2DirectionForward ;
    setDistanceToCover(abs(value)) ;
  } else if (value == 0){
    robotRotating = 0 ;
  }
  return 0 ;
}

// update system status

int calcDeltaSpeed(){
  int result = 0 ;
  robotSpeed = round(((float)motor1Speed + (float)motor2Speed)/2) ;
  int deltaSpeed = speedSetPoint - (int)robotSpeed ;
  if (deltaSpeed > 0){
    // we need to accelerate
    if (robotDirection > 0){
      result = forwardAcceleration ;
    } else {
      result = reverseAcceleration ;
    }
  }
  
  if (deltaSpeed < 0){
    // we need to decelerate 
     if (robotDirection > 0){
      result = -forwardDeceleration ;
    } else {
      result = -reverseDeceleration ;
    }   
  }
  return result;
}
