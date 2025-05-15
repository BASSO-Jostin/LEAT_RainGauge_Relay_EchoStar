/*
       __          ____        _____                       __    _ __
      / /   ____  / __ \____ _/ ___/____  ____ _________  / /   (_) /_
     / /   / __ \/ /_/ / __ `/\__ \/ __ \/ __ `/ ___/ _ \/ /   / / __ \
    / /___/ /_/ / _, _/ /_/ /___/ / /_/ / /_/ / /__/  __/ /___/ / /_/ /
   /_____/\____/_/ |_|\__,_//____/ .___/\__,_/\___/\___/_____/_/_.___/
                                /_/
   Author: mtnguyen
   This library jointy developed by UCA & RFThings

   This example continuously listens for LoRaWAN packets from sensors (or other
   LoRaWAN device) & prints packet information to Serial Monitor if it's successfully
   received. This sketch is NOT included packet integrity check, packet sender
   address filter, and repeat the received packet.

   This relay example included the sleep for low-power consumption. Disable
   U(S)ART Support in Arduino IDE Tool Menu & uncommnet
   USE_LOW_POWER_FEATURE_WITH_SLEEP option to archive low-power mode.

*/

#include <RFThings.h>
#include <radio/sx126x/rfthings_sx126x.h>
#include <STM32LowPower.h>  // https://github.com/stm32duino/STM32LowPower

#include "Project_configuration.h"

#define USE_LOW_POWER_FEATURE_WITH_SLEEP

HardwareSerial Serial2(PA3, PA2);
HardwareSerial Serial1(PB7, PB6);

#if defined(HAL_UART_MODULE_ENABLED)
#define LOG_D(...) Serial1.print(__VA_ARGS__)
#define LOG_D_LN(...) Serial1.println(__VA_ARGS__)
#else
#define LOG_D(...)
#define LOG_D_LN(...)
#endif

// Device information (For uploading relay status)
static uint8_t nwkS_key[] = { 0x2B, 0x65, 0xA2, 0x97, 0x60, 0xB9, 0x15, 0xDE, 0x10, 0x59, 0x67, 0x75, 0x16, 0xA8, 0xE9, 0x1D };  // <-- MODIFY THIS INFORMATION ACCORDING TO YOUR USECASE
static uint8_t appS_key[] = { 0x89, 0xCE, 0xC1, 0x77, 0x53, 0x32, 0x7E, 0x95, 0xE6, 0x63, 0x90, 0x4F, 0x7F, 0x04, 0xFB, 0x70 };  // <-- MODIFY THIS INFORMATION ACCORDING TO YOUR USECASE
static uint8_t dev_addr[] = { 0x26, 0x0B, 0xBD, 0xFB };                                                                          // <-- MODIFY THIS INFORMATION ACCORDING TO YOUR USECASE

rfthings_sx126x sx126x;
rft_status_t status;

// Relay params (Listen to all devices meet this LoRaPHY Params, NO FILTER)
rft_lora_params_t relay_lora_params;
byte payload[255];
uint32_t payload_len;

#define SW_VCTL1_PIN PB8
#define SW_VCTL2_PIN PC13

#define USR_LED_PIN PA9

char hexPayload[50];  // Every byte = 2 caracters hex + '\0'
uint16_t framecounter_uplink;

typedef enum {
  RF_SW_MODE_TX = 0,
  RF_SW_MODE_RX
} rf_sw_mode_t;

void setup(void) {
  pinMode(USR_LED_PIN, OUTPUT);
  digitalWrite(USR_LED_PIN, HIGH);  // Active-Low LED

  pinMode(SW_VCTL1_PIN, OUTPUT);
  pinMode(SW_VCTL2_PIN, OUTPUT);
  sw_ctrl_set_mode(RF_SW_MODE_RX);

  LowPower.begin();


  echostar_init();



#if defined(HAL_UART_MODULE_ENABLED)
  Serial1.begin(115200);
  while (!Serial && (millis() < 3000)) {
  }
#endif

  // Inititialize the LoRa Module SX126x
  status = sx126x.init(RFT_REGION_EU863_870);
  LOG_D("LoRa SX126x: ");
  LOG_D_LN(rft_status_to_str(status));
  LOG_D("Library version: ");
  LOG_D_LN(LIBRARY_VERSION);

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

  blink_led(3);


  // AT JOIN command in order to be connected to the satellite
  //Serial2.write("AT+REGION=MSS-S\r\n");
  delay(100);
  Serial2.write("AT+JOIN\r\n");
  framecounter_uplink = 0;

  delay(2000);
}

void loop(void) {
  blink_led(1);
  LOG_D_LN("Enter RELAY mode...");

#if defined(USE_LOW_POWER_FEATURE_WITH_SLEEP)
  switch (sx126x.relay(&relay_lora_params, payload, payload_len, sw_ctrl_set_mode_rx, board_sleep))
#else
  switch (sx126x.relay(&relay_lora_params, payload, payload_len, sw_ctrl_set_mode_rx, NULL))
#endif
  {
    case RFT_STATUS_OK:
      if (payload_len > 0) {
        // TODO: Filter packet and verify packet integrity

        LOG_D("[Relay] OK: Received ");
        LOG_D(payload_len);
        LOG_D(" bytes from Device Address: 0x");
        LOG_D_LN(parse_dev_id(payload, payload_len), HEX);

        LOG_D("RAW Payload: ");
        for (uint8_t i = 0; i < payload_len; i++) {
          LOG_D(payload[i]);
          LOG_D(" ");
        }
        LOG_D_LN();


        memset(hexPayload, 0, 50);
        for (int i = 0; i < 19; i++) {
          sprintf(&hexPayload[2 * i], "%02X", payload[i]);  // Convert every byte to hex
        }

        LOG_D("HexPayload : ");
        LOG_D(hexPayload);
        LOG_D_LN();





        char packet[255];
        int packet_len = 0;
        memset(packet, 0, 255);

        // To save dev_Addr in packet
        packet[packet_len++] = hexPayload[8];
        packet[packet_len++] = hexPayload[9];
        packet[packet_len++] = hexPayload[6];
        packet[packet_len++] = hexPayload[7];
        packet[packet_len++] = hexPayload[4];
        packet[packet_len++] = hexPayload[5];
        packet[packet_len++] = hexPayload[2];
        packet[packet_len++] = hexPayload[3];

        // To save payload in packet
        for (int i = 18; i < 30; i++) {
          packet[packet_len++] = hexPayload[i];
        }

        char hexFrameCounter[10];
        memset(hexFrameCounter, 0, 10);

        snprintf(hexFrameCounter, sizeof(hexFrameCounter), "%04X", framecounter_uplink);
        // To save EchoStar FrameCounter in packet
        packet[packet_len++] = hexFrameCounter[0];
        packet[packet_len++] = hexFrameCounter[1];
        packet[packet_len++] = hexFrameCounter[2];
        packet[packet_len++] = hexFrameCounter[3];

        LOG_D("To be sent : ");
        LOG_D(packet);
        LOG_D_LN();



        char command_packet[150];
        memset(command_packet, 0, 150);


        sprintf(command_packet, "AT+SEND=1,0,8,1,%s\r\n", packet);
        LOG_D_LN("To see what is sent ");
        LOG_D_LN(command_packet);

        // Sending packet with AT + SEND command to the satellite
        Serial2.println(command_packet);
        LOG_D("This is the packet which is sent : ");
        LOG_D_LN(command_packet);



        LOG_D("    Received RSSI: ");
        LOG_D_LN(relay_lora_params.rssi);
        LOG_D("    Received SNR: ");
        LOG_D_LN(relay_lora_params.snr);
        LOG_D("    Received Signal RSSI: ");
        LOG_D_LN(relay_lora_params.signal_rssi);



        framecounter_uplink += 1;
      } else {
        LOG_D_LN("[Relay] ERROR: Unknown ERROR!");
      }
      break;
    case RFT_STATUS_RX_TIMEOUT:
      LOG_D_LN("[Relay] ERROR: Preamble detected, RX timeout!");
      break;
    case RFT_STATUS_PREAMBLE_DETECT_FAIL:
      LOG_D_LN("[Relay] INFO: Nothing to relay/receive!");
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

void sw_ctrl_set_mode(rf_sw_mode_t mode) {
  if (mode == RF_SW_MODE_TX) {
    digitalWrite(SW_VCTL1_PIN, LOW);
    digitalWrite(SW_VCTL2_PIN, HIGH);
  } else {
    digitalWrite(SW_VCTL1_PIN, HIGH);
    digitalWrite(SW_VCTL2_PIN, LOW);
  }
}

void sw_ctrl_set_mode_tx(void) {
  sw_ctrl_set_mode(RF_SW_MODE_TX);
}

void sw_ctrl_set_mode_rx(void) {
  sw_ctrl_set_mode(RF_SW_MODE_RX);
}

#if defined(USE_LOW_POWER_FEATURE_WITH_SLEEP)
void board_sleep(void) {
  delay(5);

  LowPower.sleep();

  delay(5);

#if defined(HAL_UART_MODULE_ENABLED)
  Serial1.flush();
#endif
}
#endif

void blink_led(uint8_t num_of_blink) {
  while (num_of_blink > 0) {
    num_of_blink--;

    digitalWrite(USR_LED_PIN, LOW);  // Active-Low LED
    delay(50);
    digitalWrite(USR_LED_PIN, HIGH);  // Active-Low LED
    delay(150);
  }
}


void echostar_init(void) {
  Serial2.begin(115200);
  //Serial2.write("AT+TXPMSS=27\r\n");
}


void send_status_packet(void) {
  // TODO: Compose an actual status packet! It should included data like: Battery voltage, temperature, humidity, counters, etc.
  Serial2.println("AT+SEND=1,0,9,1,THIS\r\n");
}
