#ifndef Patrol_h
#define Patrol_h


enum Patrol_State{
    LEFT, 
    MIDDLE, 
    RIGHT
};
 

void patrolling(Patrol_State patrol_state)
{
    switch(patrol_state):
        case LEFT:
        {
            turn_right_90(15);

            drive_foward(15, 6)
            //if signal on --> turn right

            patrol_state = MIDDLE;
            break;
        }

        case MIDDLE:
        {
            drive_foward(15, 6);
            patrol_state = RIGHT;
            break;
        }

        case RIGHT:
        {
            turn_left_90(motorSpeed);
            drive_foward(motorSpeed, 6);

            patrol_state = MIDDLE;
        }
}

#endif