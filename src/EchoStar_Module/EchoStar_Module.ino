/* ============================== LIBRARIES ============================== */
#include <STM32LowPower.h>
#include <STM32RTC.h>

#include "Project_configuration.h"
#include "es_delay.h"
#include "es_watchdog.h"



/* ============================== MACRO ============================== */

#define RELAY_DATA_AVAILABLE_PIN PB5 // Wake_up pin activation



/* ============================== GLOBAL VARIABLES ============================== */

int acknowledgment = 0;

char buffer[255];
uint8_t buffer_len = 0;

uint16_t framecounter_uplink;
uint16_t frame_Problem;



/* ============================== MAIN ============================== */
void setup(void)
{
  // put your setup code here, to run once:
  pinMode(GNSS_PWR_ENABLE_PIN, OUTPUT);
  digitalWrite(GNSS_PWR_ENABLE_PIN, LOW);

  pinMode(SENSORS_PWR_ENABLE_PIN, OUTPUT);
  digitalWrite(SENSORS_PWR_ENABLE_PIN, LOW);

  DELAY_MANAGER.init();
  pinMode(LED_BUILTIN, OUTPUT);

#if defined(ECHOSTAR_PWR_ENABLE_PIN)
  pinMode(ECHOSTAR_PWR_ENABLE_PIN, OUTPUT);
  digitalWrite(ECHOSTAR_PWR_ENABLE_PIN, HIGH);
#endif

#if defined(DPDT_PWR_ENABLE_PIN)
  pinMode(DPDT_PWR_ENABLE_PIN, OUTPUT);
  digitalWrite(DPDT_PWR_ENABLE_PIN, HIGH);
#endif
  pinMode(DPDT_CTRL_PIN, OUTPUT);
  digitalWrite(DPDT_CTRL_PIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(RELAY_DATA_AVAILABLE_PIN), relay_data_available_io_usr, RISING);

  USB_SERIAL.begin(115200);
  while (!USB_SERIAL)
    ;

  USB_SERIAL.println("Starting...");

  ECHOSTAR_SERIAL.begin(115200);

  // Initialize the serial mode
  Serial2.begin(115200);

  // AT JOIN command in order to be connected to the satellite
  ECHOSTAR_SERIAL.write("AT+JOIN\r\n");

  digitalWrite(LED_BUILTIN, LOW);
  // EM2050_soft_sleep_enable(); //go to sleep mode
  delay(500);

  WATCHDOG.init();
  if (WATCHDOG.isResetByWatchdog())
  {
    USB_SERIAL.println("[ERROR] main::setup() | The reset was caused by WATCHDOG timeout");
  }

  analogReadResolution(12);

  framecounter_uplink = 0;

  frame_Problem = 0; // Not complete paquet received
}

void loop(void)
{

  USB_SERIAL.println("0");
  digitalWrite(LED_BUILTIN, LOW);
  EM2050_soft_sleep_disable();
  DELAY_MANAGER.delay_ms(60000);
  uint16_t bat = read_bat();
  // EM2050_soft_sleep_enable();

  delay(2000);

  char command_packet[150];
  memset(command_packet, 0, 150);

  // When Serial2 receives data from the relay
  while (Serial2.available())
  {
    acknowledgment = 1; // When data is received but the paquet is not complete
    buffer[buffer_len++] = Serial2.read();
    // To see what is received
    USB_SERIAL.println(buffer_len);

    if (buffer_len == 38)
    {
      acknowledgment = 2; // the paquet is complete
      digitalWrite(LED_BUILTIN, HIGH);

      char packet[50];
      int packet_len = 0;
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
      for (int i = 18; i < 30; i++)
      {
        packet[packet_len++] = buffer[i];
      }

      char hexFrameCounter[5];
      memset(hexFrameCounter, 0, 5);

      snprintf(hexFrameCounter, sizeof(hexFrameCounter), "%04X", framecounter_uplink);
      // To save EchoStar FrameCounter in packet
      packet[packet_len++] = hexFrameCounter[0];
      packet[packet_len++] = hexFrameCounter[1];
      packet[packet_len++] = hexFrameCounter[2];
      packet[packet_len++] = hexFrameCounter[3];

      char hexBat[5];
      memset(hexBat, 0, 5);

      snprintf(hexBat, sizeof(hexBat), "%04X", bat);

      USB_SERIAL.println(bat, HEX);

      // To save data Battery Data in packet
      packet[packet_len++] = hexBat[0];
      packet[packet_len++] = hexBat[1];
      packet[packet_len++] = hexBat[2];
      packet[packet_len++] = hexBat[3];

      char hexFrameProblem[5];
      memset(hexFrameProblem, 0, 5);

      snprintf(hexFrameProblem, sizeof(hexFrameProblem), "%04X", frame_Problem);
      // To save EchoStar Frame Problem in packet
      packet[packet_len++] = hexFrameProblem[0];
      packet[packet_len++] = hexFrameProblem[1];
      packet[packet_len++] = hexFrameProblem[2];
      packet[packet_len++] = hexFrameProblem[3];

      // Buffer for AT command in char
      char command[150];

      sprintf(command, "AT+SEND=1,0,8,0,%s\r\n", buffer);
      USB_SERIAL.println("To see what is in the buffer ");
      USB_SERIAL.println(command);
      delay(100);

      sprintf(command_packet, "AT+SEND=1,0,8,0,%s\r\n", packet);
      USB_SERIAL.println("To see what is sent ");
      USB_SERIAL.println(command_packet);

      // Sending packet with AT + SEND command to the satellite
      ECHOSTAR_SERIAL.println(command_packet);
      buffer_len = 0; // Reset the counter after sending

      framecounter_uplink += 1;

      delay(500);
    }
  }

  if (acknowledgment == 1)
  {
    Serial2.write(1); // 1 for NAK
    USB_SERIAL.println("I am 1 ");
    frame_Problem += 1;
    acknowledgment = 0;
  }
  else if (acknowledgment == 2)
  {
    Serial2.write(2); // 2 for ACK
    USB_SERIAL.println("I am 2 ");
    acknowledgment = 0;
  }

  delay(1000);

  WATCHDOG.reload();
}



/* ============================== OTHER FUNCTIONS ============================== */

void EM2050_soft_sleep_enable(void)
{
  pinMode(ECHOSTAR_RTS_PIN, OUTPUT);
  digitalWrite(ECHOSTAR_RTS_PIN, HIGH);
  delay(50);
}

void EM2050_soft_sleep_disable(void)
{
  pinMode(ECHOSTAR_RTS_PIN, INPUT);
  delay(50);
}

uint16_t read_bat(void)
{
  // uint16_t voltage_adc = (uint16_t)analogRead(SENSORS_BATERY_ADC_PIN);
  uint16_t voltage_adc = (uint16_t)analogRead(PB1);
  uint16_t voltage = (uint16_t)((ADC_AREF / 4.096) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)voltage_adc);
  return voltage;
}



/* ============================== INTERRUPTS ============================== */

void relay_data_available_io_usr(void)
{
  // INFO: This function is for waking-up the MCU only. No need to do anything here.
}

/* ============================== END ============================== */