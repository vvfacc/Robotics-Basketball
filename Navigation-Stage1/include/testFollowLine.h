#include "SimpleRSLK.h"
#include "QTRSensors.h"

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];    

void floorCalibration() {
  /* Place Robot On Floor (no line) */
  delay(2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);

  delay(1000);

  Serial.println("Running calibration on floor");
  simpleCalibrate();
  Serial.println("Reading floor values complete");

  btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  delay(1000);

  enableMotor(BOTH_MOTORS);
}

void simpleCalibrate() {
  /* Set both motors direction forward */
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  /* Enable both motors */
  enableMotor(BOTH_MOTORS);
  /* Set both motors speed 20 */
  setMotorSpeed(BOTH_MOTORS,20);

  for(int x = 0;x<100;x++){
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal,sensorMinVal,sensorMaxVal);
  }

  /* Disable both motors */
  disableMotor(BOTH_MOTORS);
}

void driveStraight(){
    if(linePos > 0 && linePos < 3000) {
    setMotorSpeed(LEFT_MOTOR,normalSpeed);
    setMotorSpeed(RIGHT_MOTOR,fastSpeed);
  } else if(linePos > 3500) {
    setMotorSpeed(LEFT_MOTOR,fastSpeed);
    setMotorSpeed(RIGHT_MOTOR,normalSpeed);
  } else {
    setMotorSpeed(LEFT_MOTOR,normalSpeed);
    setMotorSpeed(RIGHT_MOTOR,normalSpeed);
  }
  
}

void atIntersecton(){
    setMotorSpeed(BOTH_MOTORS, 0);
    numIntersection = numIntersection +1
    if numIntersection == 3{
      while true{
        rotateCCW();
      }
    } else if numIntersection == 2{
      rotateCW();
    }
  
}

void rotateCCW(){
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS, normalSpeed);
  delay(500);
  setMotorSpeed(BOTH_MOTORS, 0);
}

void rotateCW(){
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS, normalSpeed);
  delay(500);
  setMotorSpeed(BOTH_MOTORS, 0);
}

bool isIntersection(sensorVal){
  int countBlackSensors = 0;
  int blackSensorVal = 2500;
  readLineSensor(sensorVal);
  for (int x = 0;x<LS_NUM_SENSORS;x++){
    if (sensorVal[x] >= blackSensorVal -100){
      countBlackSensors = countBlackSensors + 1;
    }
  }
  if countBlackSensors >= 6{
    return true;
  }
  else{
      return false;
  }
}



void setup()
{
  Serial.begin(115200);

  setupRSLK();
  /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);
  clearMinMax(sensorMinVal,sensorMaxVal);
 
  uint16_t normalSpeed = 17;
  uint16_t fastSpeed = 24;
  int numIntersections = 0;

  uint8_t lineColor = DARK_LINE;

  floorCalibration();
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  delay(1000);
  }

void loop()
{
  //Get sensor array data and compute position
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,
            sensorCalVal,
            sensorMinVal,
            sensorMaxVal,
            lineColor);

  uint32_t linePos = getLinePosition(sensorCalVal,lineColor);

  enableMotor(BOTH_MOTORS);
  
  if isIntersecton(sensorVal){
    atIntersecton();
  }
  else{
    driveStraight();
  }
  
}