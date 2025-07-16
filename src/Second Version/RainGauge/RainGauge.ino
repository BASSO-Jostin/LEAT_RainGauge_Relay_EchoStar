/**
   '########:::::'###::::'####:'##::: ##::'######:::'##::::'##::::'###:::::'######:::'########:
    ##.... ##:::'## ##:::. ##:: ###:: ##:'##... ##:: ##:::: ##:::'## ##:::'##... ##:: ##.....::
    ##:::: ##::'##:. ##::: ##:: ####: ##: ##:::..::: ##:::: ##::'##:. ##:: ##:::..::: ##:::::::
    ########::'##:::. ##:: ##:: ## ## ##: ##::'####: ##:::: ##:'##:::. ##: ##::'####: ######:::
    ##.. ##::: #########:: ##:: ##. ####: ##::: ##:: ##:::: ##: #########: ##::: ##:: ##...::::
    ##::. ##:: ##.... ##:: ##:: ##:. ###: ##::: ##:: ##:::: ##: ##.... ##: ##::: ##:: ##:::::::
    ##:::. ##: ##:::: ##:'####: ##::. ##:. ######:::. #######:: ##:::: ##:. ######::: ########:
   ..:::::..::..:::::..::....::..::::..:::......:::::.......:::..:::::..:::......::::........::
   
   @file RainGauge.ino
   @author jostin, minhxl, binhphuong, fferrero

   @brief This sketch help controlling on-board sensors, measuring battery level. For detail description, 
   please visit: https://github.com/XuanMinh201/RainGauge

   @date 2025-16-07

*/

/*const char *sw_version = "1.0.0";
const char *model_id = "RAK11720";
const char *chip_id = "CHIP12345678";
const char *build_date = __DATE__;
const char *build_time = __TIME__;
const char *repo_info = "GitHub repo info";
const char *cli_version = "CLI v1.0";
const char *api_version = "API v1.0";*/

#include "Adafruit_SHTC3.h"
#include <stdint.h>  //http://librarymanager/All#Adafruit_SHTC3
#include "Encrypt.h"
#include "radio.h"
//#include "AES-128.h"

// Set pin number
#define buttonPin P38

// Rain Gauge battery
#define ADC_AREF 2.0f
#define BATVOLT_R1 1.0f
#define BATVOLT_R2 2.0f
#define BATVOLT_PIN A1

uint16_t voltage_adc;
uint16_t voltage;

// Set Interrupt
int ledToggle;
int previousState = HIGH;
unsigned int previousPress;
volatile int buttonFlag, buttonFlag_falseDetect;
const int buttonDebounce = 50;
volatile int lastDetect = 0;

volatile int rainFlag;     // turn on this flag if it is rain
volatile int notRainFlag;  // turn on this flag if it is not rain
volatile unsigned int rainGaugeCount = 0;
unsigned long time1 = 0;

uint32_t estimatedNextUplink = 0;
int rain_count;
// Set sensor variables
int16_t temper;
uint8_t humi;

// Rain Stop Time
uint64_t lastRain = 0;  // the last time when it was rain
uint64_t elapsedRain;
uint64_t spendTime;  // the remaining time before wake up in period OTAA

bool bucketPositionA = false;  // one of the two positions of tipping-bucket

int ABP_PERIOD = 300000;  //  sleep cycle 5m

//time in case of heavy rain.
#define t_rain (120000)



/** Packet buffer for sending */
uint8_t payload[6] = { 0 };
uint8_t lorawan_packet[19] = { 0 };
uint16_t framecounter_uplink;
static uint8_t nwkS_key[] = { 0x2B, 0x65, 0xA2, 0x97, 0x60, 0xB9, 0x15, 0xDE, 0x10, 0x59, 0x67, 0x75, 0x16, 0xA8, 0xE9, 0x1D };
static uint8_t appS_key[] = { 0x89, 0xCE, 0xC1, 0x77, 0x53, 0x32, 0x7E, 0x95, 0xE6, 0x63, 0x90, 0x4F, 0x7F, 0x04, 0xFB, 0x70 };

Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();


void setup() {
  Serial.begin(115200, RAK_AT_MODE);

  // Initialize Interrupt
  Serial.println("RAKwireless Arduino Interrupt Example");
  Serial.println("------------------------------------------------------");
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), button_ISR, FALLING);

  buttonFlag = 0;
  buttonFlag_falseDetect = 0;
  lastDetect = 0;

  analogReadResolution(10);

  // Initialize SHTC3
  Serial.println("SHTC3 test");
  if (!shtc3.begin()) {
    Serial.println("Couldn't find SHTC3");
    //while (1) delay(1);
  }
  Serial.println("Found SHTC3 sensor");

  // Wake-up
  api.system.sleep.setup(RUI_WAKEUP_FALLING_EDGE, A1);


  //P2P Parameters
  // Serial.begin(115200);
  // Exemple : Demander la version du firmware"
  if (api.lora.nwm.get() != 0) {
    Serial.printf("Set Node device work mode %s\r\n",
                  api.lora.nwm.set() ? "Success" : "Fail");
    api.system.reboot();
  }
  api.lora.pfreq.set(866600000);
  api.lora.psf.set(7);
  api.lora.pbw.set(0);
  api.lora.pcr.set(0);
  api.lora.ppl.set(976);
  api.lora.ptp.set(14);
  api.lora.syncword.set(0x3444);

  Serial.println("AT+LBT=1");  //Activate LBT Mode -> Listen Before Talk
}

void uplink_routine() {
  /** Payload of Uplink */
  uint8_t payload_len = 0;
  uint8_t packet_len = 0;

  payload[payload_len++] = (uint8_t)buttonFlag;
  payload[payload_len++] = temper >> 8;
  payload[payload_len++] = temper & 0xFF;
  payload[payload_len++] = (uint8_t)humi & 0xFF;
  payload[payload_len++] = voltage >> 8;
  payload[payload_len++] = voltage & 0xFF;

  unsigned char MAC_header = 0x40;  // Unconfirmed data up
  unsigned char FCtrl = 0x00;
  static uint8_t device_address[] = { 0x26, 0x0B, 0xBD, 0xAC };
  uint8_t tx_port = 1;

  unsigned char MIC[4];

  lorawan_packet[0] = MAC_header;
  lorawan_packet[1] = device_address[3];
  lorawan_packet[2] = device_address[2];
  lorawan_packet[3] = device_address[1];
  lorawan_packet[4] = device_address[0];
  lorawan_packet[5] = FCtrl;

  lorawan_packet[6] = framecounter_uplink & 0x00FF;
  lorawan_packet[7] = (framecounter_uplink >> 8) & 0x00FF;
  lorawan_packet[8] = tx_port;

  packet_len = 9;

  //Encrypt_Payload(payload, payload_len, appS_key , device_address, 0x00,framecounter_uplink);

  for (int i = 0; i < payload_len; i++) {
    lorawan_packet[packet_len++] = payload[i];
  }


  Construct_Data_MIC(lorawan_packet, packet_len, nwkS_key, device_address, 0x00, framecounter_uplink, MIC);


  for (int i = 0; i < 4; i++) {
    lorawan_packet[packet_len++] = MIC[i];
  }


  Serial.println(" MIC data : ");
  for (int i = 0; i < 4; i++) {
    Serial.printf("0x%02X ", MIC[i]);
  }

  Serial.println("");

  Serial.println("Frame counter");
  Serial.println(framecounter_uplink);


  Serial.println("Data Packet:");
  for (int i = 0; i < payload_len; i++) {
    Serial.printf("0x%02X ", payload[i]);
  }
  Serial.println("");

  send_LBT();
}

void loop() {
  // Read SHTC3
  sensors_event_t humidity, temp;
  shtc3.getEvent(&humidity, &temp);  // populate temp and humidity objects with fresh data
  temper = (int)(temp.temperature * 100);
  humi = (humidity.relative_humidity * 2);
  Serial.print("Sensors values : temp = ");
  Serial.print(temper / 100);
  Serial.println("deg");
  Serial.print("hum= ");
  Serial.print(humi / 2);
  Serial.println("%");


  Serial.print("Value: ");
  Serial.println(buttonFlag);
  Serial.print(" | False Detect: ");
  Serial.println(buttonFlag_falseDetect);

  //Battery variables
  voltage_adc = (uint16_t)analogRead(BATVOLT_PIN);
  voltage = (int16_t)(voltage_adc * ADC_AREF / 1.024) * (BATVOLT_R1 + BATVOLT_R2) / (BATVOLT_R2);

  Serial.print("Voltage: ");
  Serial.println(voltage);

  Serial.print("Voltage ADC: ");
  Serial.println(voltage_adc);


  // LoRaWAN Uplink
  uplink_routine();
  buttonFlag = 0;
  if (rain_count == 0) {
    ABP_PERIOD = 1800000;
    Serial.println("You send information every 30 minutes");
  } else {
    ABP_PERIOD = 180000;
    Serial.println("You send information every 3 minutes");
  }
  Serial.println(ABP_PERIOD);
  rain_count = 0;

  // Set sleep until the next LoRaWAN Uplink
  Serial.printf("Try sleep %ums..", ABP_PERIOD);
  estimatedNextUplink = millis() + ABP_PERIOD;
  api.system.sleep.all(ABP_PERIOD);

  // Re-check the wake up reason. If the wakeup caused by External Interrupt, go back to sleep mode
  while (estimatedNextUplink > millis()) {
    uint32_t remainingSleepTime = estimatedNextUplink - millis();
    api.system.sleep.all(remainingSleepTime);
  }

  Serial.println("Wakeup..");
}

void button_ISR() {
  if (rain_count == 0) {
    estimatedNextUplink = estimatedNextUplink - (ABP_PERIOD - t_rain);
  }
  rain_count++;
  int _now = millis();
  if ((_now - lastDetect) > buttonDebounce) {
    lastDetect = _now;
    buttonFlag++;
  } else {
    buttonFlag_falseDetect++;
  }
}

void send_LBT() {
  bool isFree = false;
  int max_attempts = 3;
  int attempts = 0;

  while (!isFree && attempts < max_attempts) {
    isFree = Radio.IsChannelFree(866600000, 125000, -90, 10);

    if (!isFree) {
      Serial.println("Channel is not free, retrying in 1 minute...");
      delay(60 * 1000);  // Attendre 1 minute
      attempts++;
    }
  }

  if (isFree) {
    if (api.lora.psend(sizeof(lorawan_packet), lorawan_packet)) {
      Serial.println("Sending to GateWay is succesfull");
      framecounter_uplink += 1;
    } else {
      Serial.println("Sending to GateWay failed");
    }
  } else {
    Serial.println("Failed to access channel after multiple attempts.");
  }
}
