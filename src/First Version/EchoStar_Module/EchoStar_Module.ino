/*
Author: jostin, m1nhle, mtnguyen

This program receives data from the relay via the Tx and Rx pins, and then forwards the received data to the satellite.

*/


/* ============================== LIBRARIES ============================== */
#include "Project_configuration.h"
#include "es_delay.h"
#include "es_watchdog.h"
#include "es_log.h"
#include <STM32RTC.h>


/* ============================== MACRO ============================== */

#define RELAY_SERIAL Serial2
#define RELAY_DATA_AVAILABLE_PIN PB5    // Wake_up pin activation
#define RELAY_RESET PB6                 // Pin used to reset relay
#define RELAY_RESET_PERIOD_S (30 * 60)  // Counter used to reset relay each 30m
#define STATUS_PACKET_PERIOD_S (30 * 60)


/* ============================== GLOBAL VARIABLES ============================== */

static STM32RTC &rtc = STM32RTC::getInstance();

char buffer[255];
uint8_t buffer_len = 0;

uint16_t framecounter_uplink = 0;
uint16_t frame_Problem = 0;

uint32_t send_status_timestamp_s = 0;
uint32_t relay_reset_timestamp_s = 0;

bool relay_data_available_flag = false;

bool ackReceived = false;
String inputBuffer = "";


/* ============================== MAIN ============================== */

void setup(void) {
  // Initialize es_delay library
  DELAY_MANAGER.init();

  // Initialize all STM32U5 GPIOs like LED, LDO controls, etc.
  gpio_init();

  // Initialize es_log library
  LOG.init();
  LOG.println("\n\n\n\n");
  LOG.println("======================================================");
  LOG.print("[INFO] main::setup() | System restarted at ");
  LOG.println((unsigned int)millis());

  // Set ADC read resolution to 12 bits
  analogReadResolution(12);

  echostar_init();

  // Initialize the serial for relay
  RELAY_SERIAL.begin(115200);

  // AT JOIN command in order to be connected to the satellite
  ECHOSTAR_SERIAL.write("AT+JOIN\r\n");

  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);

  WATCHDOG.init();
  if (WATCHDOG.isResetByWatchdog()) {
    LOG.println("[WARNING] main::setup() | The reset was caused by WATCHDOG timeout");
  } else {
    LOG.println("[INFO] main::setup() | The reset was caused by External Reset");
  }
  EM2050_soft_sleep_disable();

  relay_data_available_flag = false;

  rtc.setEpoch(1743681600);                             // Wednesday, February 26, 2025 4:35:00 PM GMT+01:00
  send_status_timestamp_s = rtc.getEpoch() + (2 * 60);  // After 2 mins
  relay_reset_timestamp_s = rtc.getEpoch() + (2 * 60);  // After 2 mins

  LOG.println("[INFO] main::setup() | Initialization DONE, jumping to main loop");
}


/* ============================== LOOP ============================== */

void loop(void) {
  uint32_t now_timestamp_s = rtc.getEpoch();

  LOG.println("\n\n\n\n");
  LOG.print("[INFO] main::loop() | Device wakeup, now_timestamp_s = ");
  LOG.println((unsigned int)now_timestamp_s);

  // Check if RELAY DATA is available? Send to satellite immidiately if yes.
  if (relay_data_available_flag) {
    relay_data_available_flag = false;

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1500);
    digitalWrite(LED_BUILTIN, LOW);

    LOG.println("[INFO] main::loop() | Relay data is available");

    read_data_from_relay();
  }

  // Send status packet every 30 mins
  if (now_timestamp_s >= send_status_timestamp_s) {
    send_status_timestamp_s = now_timestamp_s + STATUS_PACKET_PERIOD_S;  // Schedule the next status uplink

    LOG.print("[INFO] main::loop() | Sending status packet, next status packet is scheduled at ");
    LOG.println((unsigned int)send_status_timestamp_s);

    send_status_packet();
  } else {
    LOG.print("[INFO] main::loop() | Status timeout is not due. now_timestamp_s = ");
    LOG.print((unsigned int)now_timestamp_s);
    LOG.print("; send_status_timestamp_s = ");
    LOG.println((unsigned int)send_status_timestamp_s);
  }

  // Reset relay
  if (now_timestamp_s >= relay_reset_timestamp_s) {  // If it's more than 3h
    relay_reset_timestamp_s = now_timestamp_s + RELAY_RESET_PERIOD_S;

    LOG.print("[INFO] main::loop() | Resetting the relay, next relay reset is scheduled at ");
    LOG.println((unsigned int)relay_reset_timestamp_s);

    digitalWrite(RELAY_RESET, LOW);  // Reset the Relay
    delay(500);
    digitalWrite(RELAY_RESET, HIGH);
  } else {
    LOG.print("[INFO] main::loop() | Relay reset timeout is not due. now_timestamp_s = ");
    LOG.print((unsigned int)now_timestamp_s);
    LOG.print("; relay_reset_timestamp_s = ");
    LOG.println((unsigned int)relay_reset_timestamp_s);
  }

  // Reload WATCHDOG
  LOG.println("[INFO] main::loop() | Reloading Watchdog");
  WATCHDOG.reload();

  // Blink LED twice
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  delay(150);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);

  // Go to sleep for 10 senconds
  // EM2050_soft_sleep_enable(); // TODO: Test if we can save more power with this
  LOG.println("[INFO] main::loop() | Go to sleep for 10 seconds");
  DELAY_MANAGER.delay_ms(10 * 1000);
  EM2050_soft_sleep_disable();
}

/* ============================== OTHER FUNCTIONS ============================== */

void gpio_init(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RELAY_RESET, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(RELAY_RESET, LOW);
  delay(500);
  digitalWrite(RELAY_RESET, HIGH);


  // put your setup code here, to run once:
  pinMode(GNSS_PWR_ENABLE_PIN, OUTPUT);
  digitalWrite(GNSS_PWR_ENABLE_PIN, LOW);

  pinMode(SENSORS_PWR_ENABLE_PIN, OUTPUT);
  digitalWrite(SENSORS_PWR_ENABLE_PIN, HIGH);

  pinMode(ECHOSTAR_PWR_ENABLE_PIN, OUTPUT);
  digitalWrite(ECHOSTAR_PWR_ENABLE_PIN, LOW);
  delay(500);
  digitalWrite(ECHOSTAR_PWR_ENABLE_PIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(RELAY_DATA_AVAILABLE_PIN), relay_data_available_io_usr, RISING);
}

void echostar_init(void) {
  ECHOSTAR_SERIAL.begin(115200);
  ECHOSTAR_SERIAL.write("AT+TXPMSS=27\r\n");
}

void send_status_packet(void) {
  // TODO: Compose an actual status packet! It should included data like: Battery voltage, temperature, humidity, counters, etc.
  ECHOSTAR_SERIAL.println("AT+SEND=1,0,9,1,THIS\r\n");
}

void EM2050_soft_sleep_enable(void) {
  pinMode(ECHOSTAR_RTS_PIN, OUTPUT);
  digitalWrite(ECHOSTAR_RTS_PIN, HIGH);
  delay(50);
}

void EM2050_soft_sleep_disable(void) {
  pinMode(ECHOSTAR_RTS_PIN, INPUT);
  delay(50);
}

uint16_t read_bat(void) {
  // uint16_t voltage_adc = (uint16_t)analogRead(SENSORS_BATERY_ADC_PIN);
  uint16_t voltage_adc = (uint16_t)analogRead(PB1);
  uint16_t voltage = (uint16_t)((ADC_AREF / 4.096) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)voltage_adc);
  return voltage;
}

void read_data_from_relay(void) {
  char command_packet[150];
  memset(command_packet, 0, 150);

  // char buffer[255];
  memset(buffer, 0, 255);

  uint16_t bat = read_bat();

  RELAY_SERIAL.flush();
  delay(1000);

  // When RELAY_SERIAL receives data from the relay
  while (RELAY_SERIAL.available()) {
    buffer[buffer_len++] = RELAY_SERIAL.read();
    // To see what is received
    LOG.println(buffer_len);

    if (buffer_len == 38) {

      char packet[255];
      int packet_len = 0;
      memset(packet, 0, 255);
      // char hexBat[10];

      // To save dev_Addr in packet
      packet[packet_len++] = buffer[8];
      packet[packet_len++] = buffer[9];
      packet[packet_len++] = buffer[6];
      packet[packet_len++] = buffer[7];
      packet[packet_len++] = buffer[4];
      packet[packet_len++] = buffer[5];
      packet[packet_len++] = buffer[2];
      packet[packet_len++] = buffer[3];

      // To save payload in packet
      for (int i = 18; i < 30; i++) {
        packet[packet_len++] = buffer[i];
      }

      char hexFrameCounter[10];
      memset(hexFrameCounter, 0, 10);

      snprintf(hexFrameCounter, sizeof(hexFrameCounter), "%04X", framecounter_uplink);
      // To save EchoStar FrameCounter in packet
      packet[packet_len++] = hexFrameCounter[0];
      packet[packet_len++] = hexFrameCounter[1];
      packet[packet_len++] = hexFrameCounter[2];
      packet[packet_len++] = hexFrameCounter[3];

      char hexBat[10];
      memset(hexBat, 0, 10);

      snprintf(hexBat, sizeof(hexBat), "%04X", bat);

      // To save data Battery Data in packet
      packet[packet_len++] = hexBat[0];
      packet[packet_len++] = hexBat[1];
      packet[packet_len++] = hexBat[2];
      packet[packet_len++] = hexBat[3];

      char hexFrameProblem[10];
      memset(hexFrameProblem, 0, 10);

      snprintf(hexFrameProblem, sizeof(hexFrameProblem), "%04X", frame_Problem);
      // To save EchoStar Frame Problem in packet
      packet[packet_len++] = hexFrameProblem[0];
      packet[packet_len++] = hexFrameProblem[1];
      packet[packet_len++] = hexFrameProblem[2];
      packet[packet_len++] = hexFrameProblem[3];

      // Buffer for AT command in char
      char command[150];
      memset(command, 0, 150);

      sprintf(command, "AT+SEND=1,0,8,1,%s\r\n", buffer);
      LOG.println("To see what is in the buffer ");
      LOG.println(command);
      delay(100);

      sprintf(command_packet, "AT+SEND=1,0,8,1,%s\r\n", packet);
      LOG.println("To see what is sent ");
      LOG.println(command_packet);

      while (ECHOSTAR_SERIAL.available()) ECHOSTAR_SERIAL.read();  //Empty the Buffer
      delay(100);

      // Sending packet with AT + SEND command to the satellite
      ECHOSTAR_SERIAL.println(command_packet);
      LOG.print("This is the packet which is sent : ");
      LOG.println(command_packet);

      waitForAck();  //Checking the message has been received or not

      for (int i = 0; i < 2 && ackReceived == false; i++) {
        LOG.println("New attempt");
        while (ECHOSTAR_SERIAL.available()) ECHOSTAR_SERIAL.read();  // Empty again
        delay(1000);
        ECHOSTAR_SERIAL.println(command_packet);
        waitForAck();
      }

      // Insertion of data frame counter, battery and frame problem for debugging
      LOG.print("Data Frame Counter");
      LOG.println(hexFrameCounter);

      LOG.print("Data Battery");
      LOG.println(hexBat);

      LOG.print("Data Frame Problem");
      LOG.println(hexFrameProblem);

      buffer_len = 0;  // Reset the counter after sending

      framecounter_uplink += 1;

      delay(500);
    }
  }

}

/* ============================== INTERRUPTS ============================== */

void relay_data_available_io_usr(void) {
  // INFO: This function is for waking-up the MCU only. No need to do anything here.
  relay_data_available_flag = true;
}

/* ============================== ACKNOWLEDGMENT ============================== */

void waitForAck() {
  unsigned long startTime = millis();
  inputBuffer = "";
  ackReceived = false;

  while (millis() - startTime < 15000) {  // 15s timeout
    while (ECHOSTAR_SERIAL.available()) {
      char c = ECHOSTAR_SERIAL.read();
      inputBuffer += c;

      delay(1);

      // Line by line checking
      if (c == '\n') {
        LOG.println(inputBuffer);
        inputBuffer.trim();

        // Check if the line is the success
        if (inputBuffer.indexOf("SENT:1") != -1 && inputBuffer.indexOf("NOT_SENT:1") == -1) {
          ackReceived = true;
          LOG.println("ACK reçu : Message reçu !");
          framecounter_uplink += 1;
          return;
        }

        // Vérifie si la ligne indique un échec
        if (inputBuffer.indexOf("NOT_SENT:1") != -1 || inputBuffer.indexOf("ERROR") != -1) {
          LOG.println("ACK NON reçu : Message NON reçu !");
          frame_Problem += 1;  // Increment if not send
          return;
        }

        inputBuffer = "";  // Reinitialize for the next line
      }
    }
  }

  LOG.println("Timeout sans ACK clair");
}

/* ============================== END ============================== */