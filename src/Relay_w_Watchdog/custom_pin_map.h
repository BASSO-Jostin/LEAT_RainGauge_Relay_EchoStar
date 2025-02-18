/**
 * @file custom_pin_map.h
 * @author mtnguyen
 * @brief This file descript the pin map of the UCA_AIoT Board for using with STM32Duino Core
 * @version 1.0
 * @date 2025-02-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __CUSTOM_PIN_MAP_H__
#define __CUSTOM_PIN_MAP_H__

#include <Arduino.h>

#define LS_USER_BUTTON PC5
#define LS_LED PC8

#define LS_GPS_ENABLE PB11
#define LS_VERSION_ENABLE PC6
#define LS_GPS_V_BCKP PA6
#define SD_ON_OFF PC13

#define E22_NSS PC0
#define E22_NRST PA4
#define E22_BUSY PB12
#define E22_DIO1 PC7
#define E22_RXEN PC4

#define EchoStarActivation PA5

HardwareSerial HEADER_SERIAL(USART2);
#define HEADER_SERIAL_TX_PIN PA2 // Need to put HEADER_SERIAL.setTx(HEADER_SERIAL_TX_PIN); before HEADER_SERIAL.begin(115200);
#define HEADER_SERIAL_RX_PIN PA3 // Need to put HEADER_SERIAL.setRx(HEADER_SERIAL_RX_PIN); before HEADER_SERIAL.begin(115200);

#endif /* __CUSTOM_PIN_MAP_H__ */
