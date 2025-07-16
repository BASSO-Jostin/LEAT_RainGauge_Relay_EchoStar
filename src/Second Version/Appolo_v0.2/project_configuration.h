#ifndef PROJECT_CONFIGURATION_H
#define PROJECT_CONFIGURATION_H

/**
 * Uncomment this define to force the terminal using Sleep mode for low power consumption.
 */
//#define USING_SLEEP_MODE

/**
 * Uncomment this define to enable LOG functions to show on the USB Serial console.
 * Note: If HAL_UART_MODULE_ENABLED & USING_SLEEP_MODE are available at the same time, USB Serial Console may be malfunctioned after the first MCU sleep.
 */
 #define HAL_UART_MODULE_ENABLED


/**
 * Uncomment this define to enable LOG functions to write to the SDCard.
 */
#define USING_LOG_SD_CARD

/**
 * This define the log filename in the SDCard
 */
#define SD_LOG_FILENAME "log.txt"

/**
 * Uncomment this line to enable WATCHDOG feature
 */
#define USING_WATCHDOG

/**
 * This is the maximum time to call a reload for watchdog.
 * If no reload called in WATCHDOG_TIMEOUT_S seconds, a reset will be trigger.
 *
 * Unit: Seconds
 * Max value: 32
 */
#define WATCHDOG_TIMEOUT_S 60

/*************************************************************/

/* ----------------------------------------------------- */
/*** THIS SECTION IS FOR HARDWARE VALUE, DO NOT CHANGE ***/

#define ADC_AREF 3.3f
#define BATVOLT_R1 1.0f
#define BATVOLT_R2 2.0f

/*********************************************************/

/* ----------------------------------------------------- */
/*** THIS SECTION IS FOR MACROS OF DEFINE CHECKING, DO NOT CHANGE ***/

/**
 * @brief Check if USB is enabled.
 *
 * The USB should be disabled for low-power activities or deployment on battery.
 *
 * Comment out this section if you need the USB Serial for debugging.
 */
#if (defined(USING_SLEEP_MODE) && (defined(USBD_USE_CDC) || defined(USBD_USE_HID_COMPOSITE)))
#error Please disable USB from the Tools menu of your Arduino IDE for low-power operation.
#endif

/*********************************************************/

#endif /* PROJECT_CONFIGURATION_H */
