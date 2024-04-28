class Ports {
    /*** DRIVER JOYSTICK PORTS ***/

    static const int MAIN_JOY = 0;
    static const int SUB_JOY = 1;


    /*** SWERVE MODULE PORTS ***/

    /*  
    Swerve Modules go:
        0 1
        2 3
    */
    static const int FL_DRIVE = 0; 
    static const int FL_ROTATION = 1;
    static const int FL_CANCODER = 0; 

    static const int FR_DRIVE = 2; 
    static const int FR_ROTATION = 3;
    static const int FR_CANCODER = 1; 

    static const int BL_DRIVE = 4;
    static const int BL_ROTATION = 5;
    static const int BL_CANCODER = 2; 

    static const int BR_DRIVE = 6;
    static const int BR_ROTATION = 7;
    static const int BR_CANCODER = 3; 

    /*** SUBSYSTEM IDS ***/

    // Arm
    static const int ARM_MAIN = 8;
    static const int ARM_FOLLOWER = 9;

    // Elevator
    static const int ELEVATOR_MASTER = 10;
    static const int ELEVATOR_FOLLOWER = 11;
        
    // Wrist 
    static const int WRIST = 12; 

    // End effector
    static const int END_EFFECTOR = 13;

    // Pigeon
    static const int PIGEON = 20;
        
    static const int CANDLE = 21;
};