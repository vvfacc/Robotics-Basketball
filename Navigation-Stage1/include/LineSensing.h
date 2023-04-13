#ifndef LineSensing_h
#define LineSensing_h

//#include "SimpleRSLK.h"


void simpleCalibrate(uint16_t* sensorVal, uint16_t* sensorCalVal, uint16_t* sensorMinVal, uint16_t* sensorMaxVal)
{
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

#endif