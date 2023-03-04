


/*   HardwareConfig.h   */

/************USE THIS FILE TO SELECT THE APPLICABLE HARDWARE************/

/*****DRIVER STATION CONTROL HARDWARE*****/


/*chose the controller option*/
/*uncomment ONE*/
//#define LOGITECH_F310
#define EXTREME_3D_PRO
//#define PXN_0082




/***********DRIVETRAIN HARDWARE***********/
/*must be a 4 motor diferential drive configuration*/


/*uncomment one*/

#define SLIM_COMPETITION_DRIVETRAIN
//#define PRACTICE_DRIVETRAIN










/************************************************************/

#ifdef SLIM_COMPETITION_DRIVETRAIN
    #define SPARKMAX_CAN
    #define INVERT_DRIVETRAIN
    #define SWAP_LEFT_AND_RIGHT
    #define PNEUMATICS_HUB
#endif

#ifdef PRACTICE_DRIVETRAIN
    #define VICTOR_SPX_CAN
    //#define INVERT_DRIVETRAIN
    //#define SWAP_LEFT_AND_RIGHT
#endif







