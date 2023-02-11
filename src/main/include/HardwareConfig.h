/*   HardwareConfig.h   */

/*
#ifdef VICTOR_SPX_CAN
#undef VICTOR_SPX_CAN
#endif
#ifdef SPARKMAX_CAN
#undef SPARKMAX_CAN
#endif

#ifdef INVERT_DRIVETRAIN
#undef INVERT_DRIVETRAIN
#endif
*/

/************USE THIS FILE TO SELECT THE APPLICABLE HARDWARE************/



/***********DRIVETRAIN HARDWARE***********/
/*must be a 4 motor diferential drive configuration*/



/*uncomment ONE*/

//#define VICTOR_SPX_CAN
#define SPARKMAX_CAN


/*uncomment or comment out to reverse motor output*/

#define INVERT_DRIVETRAIN

/*uncomment to swap left and right side motors*/
#define SWAP_LEFT_AND_RIGHT