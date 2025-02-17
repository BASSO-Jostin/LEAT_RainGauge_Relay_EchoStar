/* ============================== LIBRARIES ============================== */
#include "Project_configuration.h"
#include "es_delay.h"
#include "es_watchdog.h"
#include "es_log.h"

/* ============================== MACRO ============================== */

#define RELAY_SERIAL Serial2
#define RELAY_DATA_AVAILABLE_PIN PB5 // Wake_up pin activation

/* ============================== GLOBAL VARIABLES ============================== */

int acknowledgment = 0;

char buffer[255];
uint8_t buffer_len = 0;

uint16_t framecounter_uplink;
uint16_t frame_Problem;

uint32_t send_status_timestamp = 0;

bool relay_data_available_flag = false;

/* ============================== MAIN ============================== */
void setup(void)
{
  // Initialize es_delay library
  DELAY_MANAGER.init();

  // Initialize all STM32U5 GPIOs like LED, LDO controls, etc.
  gpio_init();

  // Initialize es_log library
  LOG.init();

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
  if (WATCHDOG.isResetByWatchdog())
  {
    LOG.println("[ERROR] main::setup() | The reset was caused by WATCHDOG timeout");
  }
  EM2050_soft_sleep_disable();

  framecounter_uplink = 0;
  frame_Problem = 0;                                  // Not complete paquet received
  send_status_timestamp = millis() + (5 * 60 * 1000); // After 5 mins
  relay_data_available_flag = false;
}

#if 0 // Main version of JOSTIN
void loop(void)
{
  EM2050_soft_sleep_disable();
  DELAY_MANAGER.delay_ms(60000); // Delay / Sleep for 60s

  uint16_t bat = read_bat();
  // EM2050_soft_sleep_enable();

  delay(2000);

  char command_packet[150];
  memset(command_packet, 0, 150);

  // When RELAY_SERIAL receives data from the relay
  while (RELAY_SERIAL.available())
  {
    acknowledgment = 1; // When data is received but the paquet is not complete
    buffer[buffer_len++] = RELAY_SERIAL.read();
    // To see what is received
    LOG.println(buffer_len);

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
      LOG.println(hexBat);

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
      LOG.println("To see what is in the buffer ");
      LOG.println(command);
      delay(100);

      sprintf(command_packet, "AT+SEND=1,0,8,0,%s\r\n", packet);
      LOG.println("To see what is sent ");
      LOG.println(command_packet);

      // Sending packet with AT + SEND command to the satellite
      ECHOSTAR_SERIAL.println(command_packet);
      buffer_len = 0; // Reset the counter after sending

      framecounter_uplink += 1;

      delay(500);
    }
  }

  if (acknowledgment == 1)
  {
    RELAY_SERIAL.write(1); // 1 for NAK
    LOG.println("I am 1 ");
    frame_Problem += 1;
    acknowledgment = 0;
  }
  else if (acknowledgment == 2)
  {
    RELAY_SERIAL.write(2); // 2 for ACK
    LOG.println("I am 2 ");
    acknowledgment = 0;
  }

  delay(1000);

  WATCHDOG.reload();
}
#else // Main version of THAO
void loop(void)
{
  uint32_t now_timestamp = millis();

  // Check if RELAY DATA is available? Send to satellite immidiately if yes.
  if (relay_data_available_flag)
  {
    relay_data_available_flag = false;

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1500);
    digitalWrite(LED_BUILTIN, LOW);

    read_data_from_relay();
  }

  // Send status packet every 30 mins
  if (now_timestamp >= send_status_timestamp)
  {
    send_status_timestamp = now_timestamp + (30 * 60 * 1000); // Schedule the next status uplink
    send_status_packet();
  }
  // else if (send_status_timestamp - now_timestamp > (24 * 60 * 60 * 1000)) // If millis() is overflown, reset everything
  // {
  //   send_status_timestamp = 0;
  // }

  // Reload WATCHDOG
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
  DELAY_MANAGER.delay_ms(10 * 1000);
  EM2050_soft_sleep_disable();
}
#endif

/* ============================== OTHER FUNCTIONS ============================== */

void gpio_init(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // put your setup code here, to run once:
  pinMode(GNSS_PWR_ENABLE_PIN, OUTPUT);
  digitalWrite(GNSS_PWR_ENABLE_PIN, LOW);

  pinMode(SENSORS_PWR_ENABLE_PIN, OUTPUT);
  digitalWrite(SENSORS_PWR_ENABLE_PIN, LOW);

#if defined(ECHOSTAR_PWR_ENABLE_PIN)
  pinMode(ECHOSTAR_PWR_ENABLE_PIN, OUTPUT);
  digitalWrite(ECHOSTAR_PWR_ENABLE_PIN, LOW);
  delay(500);
  digitalWrite(ECHOSTAR_PWR_ENABLE_PIN, HIGH);
#endif

#if defined(DPDT_PWR_ENABLE_PIN)
  pinMode(DPDT_PWR_ENABLE_PIN, OUTPUT);
  digitalWrite(DPDT_PWR_ENABLE_PIN, HIGH);
#endif
  pinMode(DPDT_CTRL_PIN, OUTPUT);
  digitalWrite(DPDT_CTRL_PIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(RELAY_DATA_AVAILABLE_PIN), relay_data_available_io_usr, RISING);
}

void echostar_init(void)
{
  ECHOSTAR_SERIAL.begin(115200);
}

void send_status_packet(void)
{
  // TODO: Compose an actual status packet! It should included data like: Battery voltage, temperature, humidity, counters, etc.
  ECHOSTAR_SERIAL.println("AT+SEND=1,0,9,0,THIS\r\n");
}

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

void read_data_from_relay(void)
{
  char command_packet[150];
  memset(command_packet, 0, 150);

  uint16_t bat = read_bat();

  // When RELAY_SERIAL receives data from the relay
  while (RELAY_SERIAL.available())
  {
    acknowledgment = 1; // When data is received but the paquet is not complete
    buffer[buffer_len++] = RELAY_SERIAL.read();
    // To see what is received
    LOG.println(buffer_len);

    if (buffer_len == 38)
    {
      acknowledgment = 2; // the paquet is complete

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
      LOG.println(hexBat);

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
      LOG.println("To see what is in the buffer ");
      LOG.println(command);
      delay(100);

      sprintf(command_packet, "AT+SEND=1,0,8,0,%s\r\n", packet);
      LOG.println("To see what is sent ");
      LOG.println(command_packet);

      // Sending packet with AT + SEND command to the satellite
      ECHOSTAR_SERIAL.println(command_packet);
      buffer_len = 0; // Reset the counter after sending

      framecounter_uplink += 1;

      delay(500);
    }
  }

  if (acknowledgment == 1)
  {
    RELAY_SERIAL.write(1); // 1 for NAK
    LOG.println("I am 1 ");
    frame_Problem += 1;
    acknowledgment = 0;
  }
  else if (acknowledgment == 2)
  {
    RELAY_SERIAL.write(2); // 2 for ACK
    LOG.println("I am 2 ");
    acknowledgment = 0;
  }

  delay(1000);
}

/* ============================== INTERRUPTS ============================== */

void relay_data_available_io_usr(void)
{
  // INFO: This function is for waking-up the MCU only. No need to do anything here.
  relay_data_available_flag = true;
}

/* ============================== END ============================== */