/*
 *     __          ____        _____                       __    _ __
 *    / /   ____  / __ \____ _/ ___/____  ____ _________  / /   (_) /_
 *   / /   / __ \/ /_/ / __ `/\__ \/ __ \/ __ `/ ___/ _ \/ /   / / __ \
 *  / /___/ /_/ / _, _/ /_/ /___/ / /_/ / /_/ / /__/  __/ /___/ / /_/ /
 * /_____/\____/_/ |_|\__,_//____/ .___/\__,_/\___/\___/_____/_/_.___/
 *                              /_/
 * Author: jostin, m1nhle, mtnguyen
 * Lib jointy developed by UCA & RFThings
 */

#include <RFThings.h>
#include <rfthings_sx126x.h>
//#include <RTC.h>
#include <time.h>


// Device information (For uploading relay status)
static uint8_t nwkS_key[] = { 0x2B, 0x65, 0xA2, 0x97, 0x60, 0xB9, 0x15, 0xDE, 0x10, 0x59, 0x67, 0x75, 0x16, 0xA8, 0xE9, 0x1D };  // <-- MODIFY THIS INFORMATION ACCORDING TO YOUR USECASE
static uint8_t appS_key[] = { 0x89, 0xCE, 0xC1, 0x77, 0x53, 0x32, 0x7E, 0x95, 0xE6, 0x63, 0x90, 0x4F, 0x7F, 0x04, 0xFB, 0x70 };  // <-- MODIFY THIS INFORMATION ACCORDING TO YOUR USECASE
static uint8_t dev_addr[] = { 0x26, 0x0B, 0xBD, 0xFB };                                                                          // <-- MODIFY THIS INFORMATION ACCORDING TO YOUR USECASE

// DKPlatium
rfthings_sx126x sx126x(E22_NSS, E22_NRST, E22_BUSY, E22_DIO1, E22_RXEN);
rft_status_t status;
rft_status_t status_send_to_GW;

#define EchoStarActivation 15  //PA5 Pin used to wake up EchoStar Module

// Relay params (Listen to all devices meet this LoRaPHY Params, NO FILTER)
rft_lora_params_t relay_lora_params;
rft_lora_params_t send_toGW_lora_params;
byte payload[255];
uint32_t payload_len;

char hexPayload[50];  // Every byte = 2 caracters hex + '\0'

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);


  pinMode(LS_GPS_ENABLE, OUTPUT);
  pinMode(LS_VERSION_ENABLE, OUTPUT);
  pinMode(LS_GPS_V_BCKP, OUTPUT);
  pinMode(SD_ON_OFF, OUTPUT);


  // Inititialize the LoRa Module SX126x
  status = sx126x.init(RFT_REGION_EU863_870);
  // Serial.print("LoRa SX126x: ");
  // Serial.println(rft_status_to_str(status));

  // Set up LoRaWAN key (ABP by default)
  sx126x.set_lorawan_activation_type(RFT_LORAWAN_ACTIVATION_TYPE_ABP);
  sx126x.set_application_session_key(appS_key);
  sx126x.set_network_session_key(nwkS_key);
  sx126x.set_device_address(dev_addr);
  sx126x.set_rx1_delay(1000);

  // Setting Relay LoRa Params
  relay_lora_params.frequency = 866600000;
  relay_lora_params.spreading_factor = RFT_LORA_SPREADING_FACTOR_7;
  relay_lora_params.bandwidth = RFT_LORA_BANDWIDTH_125KHZ;
  relay_lora_params.coding_rate = RFT_LORA_CODING_RATE_4_5;
  relay_lora_params.syncword = RFT_LORA_SYNCWORD_PUBLIC;
  relay_lora_params.relay_sleep_interval_us = 1E6;  // 1 second
  relay_lora_params.relay_rx_symbol = 5;
  relay_lora_params.relay_max_rx_packet_length = 120;

  //Initialize the Serial mode
  Serial3.begin(115200);
  //Serial.begin(115200);

  pinMode(EchoStarActivation, OUTPUT);

  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop(void) {
  //sx126x.set_frequency(866600000);
  //Serial.println("Loop");

  digitalWrite(EchoStarActivation, LOW);  //Put the the module EchoStar on sleep

  switch (sx126x.relay(&relay_lora_params, payload, payload_len, NULL, /*NULL*/ sleepRelay)) {
    case RFT_STATUS_OK:
      if (payload_len > 0) {
        //Serial.begin(115200);

        digitalWrite(EchoStarActivation, HIGH);  //Wake up the module EchoStar
        delay(1000);                             //Delay before forwarding

        sx126x.set_tx_power(14);


        memset(hexPayload, 0, 50);
        for (int i = 0; i < 19; i++) {
          sprintf(&hexPayload[2 * i], "%02X", payload[i]);  // Convert every byte to hex
        }

        //The forward to the EchoStar module by using the serial mode
        Serial3.write(hexPayload);

        //Serial.println("Received");

        //Waiting the aknowledge from EchoStar Module
        while (!Serial3.available())
          ;
        int lecture = Serial3.read();

        //if we receive NAK from EchoStar, we send 5 times and if the paquet was not complete too, we continue
        for (int i = 0; i < 5; i++) {
          if (lecture == 1) {  //1 for NAK
            Serial3.write(hexPayload);
            delay(1000);
            while (!Serial3.available())
              ;
            lecture = Serial3.read();
          }

          else if (lecture == 2) {
            break;
          }
        }


        if (lecture == 2) {  //2 for ACK
          digitalWrite(LED_BUILTIN, LOW);
          //Serial.println("Received acknowledgment");

          //Blinking to show that the packet was complete
          digitalWrite(LED_BUILTIN, HIGH);
          delay(125);
          digitalWrite(LED_BUILTIN, LOW);
          delay(50);
          digitalWrite(LED_BUILTIN, HIGH);
          delay(125);
          digitalWrite(LED_BUILTIN, LOW);
        }


        delay(1000);
      } else {
        // Serial.println("[Relay] ERROR: Unknown ERROR!");
      }
      break;
    case RFT_STATUS_RX_TIMEOUT:
      // Serial.println("[Relay] ERROR: Preamble detected, RX timeout!");
      break;
    case RFT_STATUS_PREAMBLE_DETECT_FAIL:
      // Serial.println("[Relay] INFO: Nothing to relay/receive!");
      break;
    default:
      break;
  }
}

uint32_t parse_dev_id(byte *payload, uint8_t len) {
  if (payload == NULL)
    return 0;
  if (len < 11)
    return 0;  // ToDo: Verify the minimum size of a LoRaWAN Packet

  return ((payload[1]) | (payload[2] << 8) | (payload[3] << 16) | (payload[4] << 24));
}

void sleepRelay() {
  digitalWrite(LS_GPS_ENABLE, LOW);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, LOW);
  digitalWrite(SD_ON_OFF, LOW);

  SPI.end();
  delay(10);
  STM32.stop();

  SPI.begin();
  Serial3.begin(115200);
  //Serial.begin(115200);
  Serial3.flush();
}
