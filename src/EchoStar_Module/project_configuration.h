#ifndef PROJECT_CONFIGURATION_H
#define PROJECT_CONFIGURATION_H

/**
 * Uncomment this define to force the terminal using Sleep mode for low power consumption.
 */
#define USING_SLEEP_MODE

/**
 * Uncomment this define to enable LOG functions to show on the USB Serial console.
 * Note: If USING_LOG & USING_SLEEP_MODE are available at the same time, USB Serial Console may be malfunctioned after the first MCU sleep.
 */
 // #define USING_LOG

/*************************************************************/

/* ----------------------------------------------------- */
/*** THIS SECTION IS FOR HARDWARE VALUE, DO NOT CHANGE ***/

#define ADC_AREF 3.3f
#define BATVOLT_R1 1.0f
#define BATVOLT_R2 2.0f

/*********************************************************/

/* ----------------------------------------------------- */
/*** THIS SECTION IS FOR MACROS OF DEFINE CHECKING, DO NOT CHANGE ***/


/*********************************************************/

#endif /* PROJECT_CONFIGURATION_H */
