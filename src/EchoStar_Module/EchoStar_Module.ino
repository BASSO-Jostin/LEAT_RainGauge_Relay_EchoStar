@author jostin

#include "STM32LowPower.h"
#include <STM32RTC.h>
#include "es_delay.h"
#include <SoftwareSerial.h>

#define EchoStar_Activation PB5  //Wake_up pin activation

int acknowledgment = 0;

#define SWITCH_REVERSING_CONTROL_DEFAULT_VALUE true
volatile int switch_reversing_control = SWITCH_REVERSING_CONTROL_DEFAULT_VALUE;

char buffer[255];
uint8_t buffer_len = 0;


void setup(void) {
  // put your setup code here, to run once:
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

  pinMode(ECHOSTAR_SWCTRL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHOSTAR_SWCTRL_PIN), swctrl_change_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EchoStar_Activation), button_1_isr, RISING);

  //USB_SERIAL.begin(115200);
  /*while (!USB_SERIAL)
    ;*/

  //USB_SERIAL.println("Starting...");

  do_switch_ctrl_update();

  ECHOSTAR_SERIAL.begin(115200);

  //Initialize the serial mode
  Serial2.begin(115200);

  //AT JOIN command in order to be connected to the satellite
  ECHOSTAR_SERIAL.write("AT+JOIN\r\n");

  digitalWrite(LED_BUILTIN, LOW);
  //EM2050_soft_sleep_enable(); //go to sleep mode
  delay(500);
}

void loop(void) {

  USB_SERIAL.println("0");
  digitalWrite(LED_BUILTIN, LOW);
  EM2050_soft_sleep_enable();
  DELAY_MANAGER.delay_ms(60000);
  EM2050_soft_sleep_disable();
  digitalWrite(LED_BUILTIN, HIGH);


  char command_packet[150];
  memset(command_packet, 0, 150);

  //When Serial2 receives data from the relay
  while (Serial2.available()) {
    acknowledgment = 1;  //When data is received but the paquet is not complete
    buffer[buffer_len++] = Serial2.read();
    //To see what is received
    //USB_SERIAL.println(buffer_len);

    if (buffer_len == 38) {
      acknowledgment = 2;  //the paquet is complete

      char packet[50];
      int packet_len = 0;

      //To save dev_Addr in packet
      packet[packet_len++] = buffer[8];
      packet[packet_len++] = buffer[9];
      packet[packet_len++] = buffer[6];
      packet[packet_len++] = buffer[7];
      packet[packet_len++] = buffer[4];
      packet[packet_len++] = buffer[5];
      packet[packet_len++] = buffer[2];
      packet[packet_len++] = buffer[3];

      //To save payload in packet
      for (int i = 18; i < 30; i++) {
        packet[packet_len++] = buffer[i];
      }

      // Buffer for AT command in char
      char command[150];

      sprintf(command, "AT+SEND=1,0,8,0,%s\r\n", buffer);
      USB_SERIAL.println("To see what is in the buffer ");
      USB_SERIAL.println(command);
      delay(100);

      sprintf(command_packet, "AT+SEND=1,0,8,0,%s\r\n", packet);
      USB_SERIAL.println("To see what is sent ");
      USB_SERIAL.println(command_packet);

      //Sending packet with AT + SEND command to the satellite
      ECHOSTAR_SERIAL.println(command_packet);
      buffer_len = 0;  //Reset the counter after sending


      delay(500);
    }
  }

  if (acknowledgment == 1) {
    Serial2.write(1);  //1 for NAK
    USB_SERIAL.println("I am 1 ");
    acknowledgment=0;
  } 
  else if (acknowledgment == 2) {
    Serial2.write(2);  //2 for ACK
    USB_SERIAL.println("I am 2 ");
    acknowledgment=0;
  }

  delay(1000);
}


void swctrl_change_isr(void) {

  do_switch_ctrl_update();
}

void do_switch_ctrl_update(void) {
  if (digitalRead(ECHOSTAR_SWCTRL_PIN) == LOW) {

    digitalWrite(DPDT_CTRL_PIN, switch_reversing_control ? HIGH : LOW);
    digitalWrite(LED_BUILTIN, switch_reversing_control ? HIGH : LOW);
  } else {
    digitalWrite(DPDT_CTRL_PIN, switch_reversing_control ? LOW : HIGH);
    digitalWrite(LED_BUILTIN, switch_reversing_control ? LOW : HIGH);
  }
}

void sleepEchoStar(int sleeping_time_ms) {
  if (sleeping_time_ms < 1000) {
    delay(1000);
  } else {
    // Put the board to sleep
    EM2050_soft_sleep_enable();
    //mcu_sleep(sleeping_time_ms * 1000);
    EM2050_soft_sleep_disable();
  }
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

void Interruption_delay() {

  //USB_SERIAL.println("Je suis ici");
}

/*void mcu_sleep(uint32_t sleep_duration_s) {
  rtc.setAlarmEpoch(rtc.getEpoch() + sleep_duration_s);
  LowPower.deepSleep();
}*/

void button_1_isr(void) {
}
