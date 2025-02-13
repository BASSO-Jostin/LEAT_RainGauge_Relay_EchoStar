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
 //#define USING_LOG

/**
 * Default terminal behaviour is to join the satellite network for 1 minute. If satellite network join failed, move to terrestrial network.
 *
 * Uncomment either following defines to force the terminal to use Satellite / Terrestrial LoRaWAN network and skipping the other.
 * Note: Do not uncomment both at the same time.
 */
 #define FORCE_USING_SATELLITE_NETWORK
//#define FORCE_USING_TERRESTRIAL_NETWORK

/**
 * This is the time (not including camera reading) between data uplink when fire DO detected on camera.
 * Unit: Seconds
 */
//#define FIRE_DETECTED_PACKET_SENDING_PERIOD_S (5 * 60)

/**
 * This is the time (not including camera reading) between data uplink when fire DO NOT detected on camera.
 * Unit: Seconds
 */
//#define NO_FIRE_PACKET_SENDING_PERIOD_S (30 * 60)

/**
 * These are the start & stop time for the camera to be active in a day.
 *
 * Format: HHMM
 */
//#define CAMERA_ACTIVE_TIME_START_HH_MM (0530) // 05H30
//#define CAMERA_ACTIVE_TIME_STOP_HH_MM (2200)  // 22H00

/**
 * This is the time to update GNSS data.
 *
 * Note: GNSS Data is force to be available when setup unless DEBUGGING_OPTION_GNSS_DISABLE is defined.
 *
 * Unit: Seconds
 */
//#define GNSS_UPDATING_PERIOD_S (12 * 60 * 60)

/**
 * This is the that the terminal waiting (without reading data) after power on the GNSS.
 * The purpose of this waiting is for the GNSS becoming statble.
 *
 * Unit: Seconds
 */
//#define GNSS_WAITING_TIME_S (30)

/**
 * This is the maximum time to read GNSS data without force_fix_reading option.
 *
 * If the reading is longer than this option, the GNSS Data remain the same as previous. Except for
 * num_of_satellite_in_view=0 to indicate that this GNSS Data is old.
 *
 * Unit: Seconds
 */
//#define GNSS_MAX_READING_TIME_S (3 * 60)

/**
 * This is time maximum time to schedule an uplink packet.
 *
 * The EM2050 scheduling packet function could be failed due to multiple reason like:
 * MAC Command sending required (causing the maximum packet length reduce), MAC Layer Busy, etc.
 *
 * If scheduling is failed, a re-scheduling is required.
 *
 * Unit: Seconds
 */
//#define EM2050_SENDING_TIMEOUT_S (30) // This is the maximum waiting time for a successful uplink schedule in seconds

/**
 * This is the waiting time between uplink scheduling retries.
 *
 * Unit: Seconds
 */
//#define EM2050_SENDING_RETRY_TIME_S (5) // This is the time between uplink retry

/**
 * Uncomment this to disable integrated LED after initialization.
 */
// #define DISABLE_BUILTIN_LED_AFTER_INIT // Disable the on-board LED after initialization

//#define CAMERA_READ_BUFFER_SIZE 128
//#define NUM_OF_CAM_FRAME 24

/**
 * This is the probability threshold to be identified as "Fire detected" for a camera subframe.
 *
 * Range: 0 - 1
 * Type: Float
 */
//#define CAMERA_FIRE_PROBABILITY_THRESHOLD 0.9

/**
 * This is the time for the terminal to sleep between camera reading
 *
 * Unit: Seconds
 */
//#define TIME_BETWEEN_CAMERA_READ_S (60)

/**
 * This is the maximum time to read camera data.
 *
 * Unit: Seconds.
 */
//#define CAMERA_MAX_READING_TIME_S (60)

/**
 * Uncomment this line to enable WATCHDOG feature
 */
//#define USING_WATCHDOG

/**
 * This is the maximum time to call a reload for watchdog.
 * If no reload called in WATCHDOG_TIMEOUT_S seconds, a reset will be trigger.
 *
 * Unit: Seconds
 * Max value: 32
 */
//#define WATCHDOG_TIMEOUT_S 30

/* --------------------------------------------------------- */
/*** THIS SECTION IS RESTRICTED FOR DEBUGGING PURPOSE ONLY ***/
/*** All defines in this section should be commented for actual deployment ***/

// #define DEBUGGING_OPTION_GNSS_DISABLE
// #define DEBUGGING_OPTION_NO_WAITING_FOR_NETWORK_TO_JOIN
// #define DEBUGGING_OPTION_I_HAVE_NO_CAMERA

/*************************************************************/

/* ----------------------------------------------------- */
/*** THIS SECTION IS FOR HARDWARE VALUE, DO NOT CHANGE ***/

#define ADC_AREF 3.3f
#define BATVOLT_R1 1.0f
#define BATVOLT_R2 2.0f

/*********************************************************/

/* ----------------------------------------------------- */
/*** THIS SECTION IS FOR MACROS OF DEFINE CHECKING, DO NOT CHANGE ***/

#if (defined(FORCE_USING_SATELLITE_NETWORK) && defined(FORCE_USING_TERRESTRIAL_NETWORK))
#error FORCE_USING_SATELLITE_NETWORK & FORCE_USING_TERRESTRIAL_NETWORK, please choose either of them and try again.
#endif

#if (CAMERA_ACTIVE_TIME_START_HH_MM > CAMERA_ACTIVE_TIME_STOP_HH_MM)
#error CAMERA_ACTIVE_TIME_START_HH_MM is after CAMERA_ACTIVE_TIME_STOP_HH_MM, please check and try again.
#endif

#if ((CAMERA_ACTIVE_TIME_START_HH_MM % 100 >= 60) || (CAMERA_ACTIVE_TIME_START_HH_MM / 100 >= 24))
#error CAMERA_ACTIVE_TIME_START_HH_MM format is wrong.
#endif

#if ((CAMERA_ACTIVE_TIME_STOP_HH_MM % 100 >= 60) || (CAMERA_ACTIVE_TIME_STOP_HH_MM / 100 >= 24))
#error CAMERA_ACTIVE_TIME_STOP_HH_MM format is wrong.
#endif

/*********************************************************/

#endif /* PROJECT_CONFIGURATION_H */
