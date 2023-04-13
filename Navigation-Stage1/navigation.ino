#include "SimpleRSLK.h"

#define NUMBER_OF_DATA_COLLECTED 85 
enum Robot_States{
    POSITION_CALIBRATION,
    MOVING_TO_PATROL_POINT,
    PATROLLING,
    AIMMING,
    SHOOTING
};

enum Patrol_States{
  LEFT,
  MIDDLE,
  RIGHT
};

//SETTING UP CURRENT STATE
Robot_States current_robot_state = POSITION_CALIBRATION;

//SETTING UP PATROLING STATE
Patrol_States current_patrol_state = MIDDLE;

//SETTINGUP VARIABLES FOR MOVING SYSTEM
const uint8_t motorSpeed = 20;
//const uint8_t calibrationMotorSpeed = 14;

//SETTING UP SENSORS FOR FOLLOWING LINE SYSTEM
bool isCalibrationComplete = false;
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];


// SETTING ARRAY OF BUMP SWITCH
bool bp_sw[6] = {1, 1, 1, 1, 1, 1};
//SETTING UP SENSORS FOR IR SENSING SYSTEM

//SETTING UP SENSORS FOR ULTRASONIC SYSTEM
//float DistanceArr[NUMBER_OF_DATA_COLLECTED];
bool isRotated = false;
const uint8_t trigPin = 40;
const uint8_t echoPin = 39;

void setup()
{
    Serial.begin(115200);

    setupRSLK();
    /* Left button on Launchpad */
    setupWaitBtn(LP_LEFT_BTN);
    /* Red led in rgb led */
    setupLed(RED_LED);
    clearMinMax(sensorMinVal, sensorMaxVal);

    /*Setting up the UltraSonic Sensor Module*/
    //Define inputs and outputs
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT_PULLUP);

    // Set Number of intersection
    uint8_t numIntersection = 0;
}

//FUNCTION RORATING THE ROBOT AND RECORDING THE DATA FROM THE SENSOR
float* position_calibration_data_collection()
{
    delay(1000);
    String btnMsg1 = "Push left button on Launchpad to start the operation.\n";
    btnMsg1 += "Place the robot at random position\n";

    /* Wait until button is pressed to start robot */
    waitBtnPressed(LP_LEFT_BTN, btnMsg1, RED_LED);
    delay(1000);

    //Start rotating and record the data
    // Rotating ...
    static float DistanceArr[NUMBER_OF_DATA_COLLECTED];

    for(uint8_t i = 0; i < NUMBER_OF_DATA_COLLECTED; i += 1)
    {
        //Recording ...
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        // Read the echo pin to determine the time it took for the sound to travel to the object and back
        pinMode(echoPin, INPUT_PULLUP);
        float duration = pulseIn(echoPin, HIGH);
        DistanceArr[i] = (duration / 2) / 29.1;
        Serial.println(DistanceArr[i]);

        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorSpeed(BOTH_MOTORS, 12);
        enableMotor(BOTH_MOTORS);
    
    
        delay(100);
    }

    //After rotaing
    disableMotor(BOTH_MOTORS);

    
    return DistanceArr;
}

//FUNCTION GET TO THE WALL WITH MINIMUM DISTANCE
void minimun_distance_pos(float* DistanceArr)
{
    // Find the minimun distance
    float minDistance = DistanceArr[0];
    for(uint8_t i = 0; i < sizeof(DistanceArr); i++)
    {
      //distance is not 0!
      if((DistanceArr[i] != 0) && (DistanceArr[i] < minDistance))
      {
          minDistance = DistanceArr[i];
      }
    }

    // Rotate the robot to min Distance and move forward
    for(uint8_t i = 0; i < NUMBER_OF_DATA_COLLECTED; i += 1)
    {
        //Recording ...
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        // Read the echo pin to determine the time it took for the sound to travel to the object and back
        pinMode(echoPin, INPUT_PULLUP);
        float duration = pulseIn(echoPin, HIGH);
        //DistanceArr[i] = (duration / 2) / 29.1;
        if(minDistance -((duration / 2) / 29.1) <= 10)
        {

            setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
            setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
            setMotorSpeed(BOTH_MOTORS, 12);
            enableMotor(BOTH_MOTORS);
      
      
          delay(100);
        } else {
            setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
            setMotorSpeed(BOTH_MOTORS, 12);
            enableMotor(BOTH_MOTORS);

          // Stop of BumpSwitch on
          for(uint8_t j = 0; j < 6; j ++)
          {
              if (bp_sw[j] == 0)
              {
                disableMotor(BOTH_MOTORS);
              }
          }
        }
    }

}

// FUNCTION TURN RIGHT OR LEFT UNTIL REACH THE DARK LINE
void get_to_darkline()
{
    
}

bool isIntersection(uint16_t* sensorVal)
{
    uint16_t countBlackSensors = 0;
    uint16_t blackSensorVal = 2500;

    for (int x = 0;x<LS_NUM_SENSORS;x++)
    {
        if (sensorVal[x] >= blackSensorVal -100){
            countBlackSensors = countBlackSensors + 1;
        }
    }
    if (countBlackSensors >= 6)
    {
        return true;
    }
    else
    {
        return false;
    }

}


//FUNCTION PERFROM FLOOR CALIBRATION
void floorCalibration() {
    /* Place Robot On Floor (no line) */
    delay(2000);
    String btnMsg = "Push left button on Launchpad to begin calibration.\n";
    btnMsg += "Make sure the robot is on the floor away from the line.\n";
    /* Wait until button is pressed to start robot */
    waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED);

    delay(1000);

    Serial.println("Running calibration on floor");
    simpleCalibrate();
    Serial.println("Reading floor values complete");

    // btnMsg = "Push left button on Launchpad to begin line following.\n";
    // btnMsg += "Make sure the robot is on the line.\n";
    // /* Wait until button is pressed to start robot */
    // waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED);
    // delay(1000);

    // enableMotor(BOTH_MOTORS);
} 

// FUNCTION SIMPLE CALIBRATE DO CALIBRATE THE WHITE LINES
void simpleCalibrate() {
    /* Set both motors direction forward */
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    /* Enable both motors */
    enableMotor(BOTH_MOTORS);
    /* Set both motors speed 20 */
    setMotorSpeed(BOTH_MOTORS, 20);

    for (int x = 0; x < 100; x++) {
      readLineSensor(sensorVal);
      setSensorMinMax(sensorVal, sensorMinVal, sensorMaxVal);
    }

    /* Disable both motors */
    disableMotor(BOTH_MOTORS);
}


void loop()
{
    
    /* Run this setup only once */
    if (isCalibrationComplete == false) {
      floorCalibration();
      isCalibrationComplete = true;
    }

    // VARIABLES SPEED WHEN START MOVING
    uint8_t normalSpeed = 17;
    uint8_t fastSpeed = 24;

    /* Valid values are either:
      DARK_LINE  if your floor is lighter than your line
      LIGHT_LINE if your floor is darker than your line
    */
    uint8_t lineColor = DARK_LINE;


    switch(current_robot_state)
    {
        case POSITION_CALIBRATION:
        {
          //In this state, the robot have to fin[NUMBER_OF_DATA_COLLECTEDd it own position with the ultrasonic sensor
          float* DistanceArr = position_calibration_data_collection();
          //minimun_distance_pos(DistanceArr[NUMBER_OF_DATA_COLLECTED]); ???
          //get_to_darkline();
          disableMotor(BOTH_MOTORS);
          current_robot_state = MOVING_TO_PATROL_POINT;
          
          break;
        }

        case MOVING_TO_PATROL_POINT:
        {
            //In this case, the robot will move exact to the middle 2nd intersection 
            // In the first stage, the robot is alrealy at the middle line
            //Get sensor array data and compute position
            readLineSensor(sensorVal);
            readCalLineSensor(sensorVal,
                              sensorCalVal,
                              sensorMinVal,
                              sensorMaxVal,
                              lineColor);

            uint32_t linePos = getLinePosition(sensorCalVal, lineColor);
            uint8_t numIntersection = 0;
            Serial.println(normalSpeed);

            // Serial.println(sensorVal[0]); //the left-most sensor if facing same direction as robot
            // Serial.println(sensorVal[1]);
            // Serial.println(sensorVal[2]);
            // Serial.println(sensorVal[3]);
            // Serial.println(sensorVal[4]);
            // Serial.println(sensorVal[5]);
            // Serial.println(sensorVal[6]);
            // Serial.println(sensorVal[7]); //the right-most sensor if facing same direction as robot
            // Serial.println("---------------");

            // Serial.println(linePos);
            enableMotor(BOTH_MOTORS);
            if (numIntersection ==2)
            {
                disableMotor(BOTH_MOTORS);
                current_robot_state = PATROLLING;
            }

            if (linePos > 0 && linePos < 3000) 
            {
            setMotorSpeed(LEFT_MOTOR, normalSpeed);
            setMotorSpeed(RIGHT_MOTOR, fastSpeed);
            } else if (linePos > 3500) 
            {
              setMotorSpeed(LEFT_MOTOR, fastSpeed);
              setMotorSpeed(RIGHT_MOTOR, normalSpeed);
            } else 
            {
              setMotorSpeed(LEFT_MOTOR, normalSpeed);
              setMotorSpeed(RIGHT_MOTOR, normalSpeed);
            }

            if(isIntersection(sensorVal)){
                numIntersection += 1;
                
            }
        }

        case PATROLLING:
        {
            // In this case, the robot will patrol on the 2 nd line so that and detect IR Sensor
            switch(current_patrol_state)
            {
              case LEFT:
              {
                  break;
              }

              case MIDDLE:
              {
                  // Detect if the IR Sensor is working
                  break;
              }

              case RIGHT:
              {
                  break;
              }
            };
        }

        case AIMMING:
        { 
            break;
            
        }

        case SHOOTING:
        {
          break;
        }
    }
        

}
