/*
  IWatchdog + Button

  This example code is in the public domain.

  The code demonstrates the use of a independent watchdog timer.
  The watchdog timer is initialized with timeout of 10 seconds.
  Every time the button is pressed, the watchdog timer is reset.
  If left unattended the system would reset itself about every 10 seconds.

  You would have to keep pressing the button frequently (< 10 seconds apart)
  to prevent the system from resetting itself.

  You would recognize the reset condition when the LED blinks few times quickly.

  This is not a practical example, in real code you would reset the watchdog
  timer in the main loop without requiring user input.

  The code is modified version of the code from:
  http://www.arduino.cc/en/Tutorial/Button
*/

#include "custom_pin_map.h"
#include <IWatchdog.h>

const int buttonPin = LS_USER_BUTTON;
const int ledPin = LS_LED;

static int default_buttonState = LOW;

void setup(void)
{
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  // Header Serial to commincate with EchoStar
  init_header_serial();

  if (IWatchdog.isReset(true))
  {
    // LED blinks to indicate reset
    for (uint8_t idx = 0; idx < 5; idx++)
    {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
  }

  // Read default state of the pushbutton
  default_buttonState = digitalRead(buttonPin);

  // Init the watchdog timer with 10 seconds timeout
  IWatchdog.begin(10000000);
  // or with a 2 seconds window
  // IWatchdog.begin(10000000, 2000000);

  if (!IWatchdog.isEnabled())
  {
    // LED blinks indefinitely
    while (1)
    {
      digitalWrite(ledPin, HIGH);
      delay(500);
      digitalWrite(ledPin, LOW);
      delay(500);
    }
  }
}

void loop(void)
{
  // Compare current button state of the pushbutton value:
  if (digitalRead(buttonPin) == default_buttonState)
  {
    digitalWrite(ledPin, LOW);
  }
  else
  {
    digitalWrite(ledPin, HIGH);

    // Uncomment to change timeout value to 6 seconds
    // IWatchdog.set(6000000);

    // Reload the watchdog only when the button is pressed
    IWatchdog.reload();
  }
}

void init_header_serial(void)
{
  HEADER_SERIAL.setTx(HEADER_SERIAL_TX_PIN);
  HEADER_SERIAL.setRx(HEADER_SERIAL_RX_PIN);
  HEADER_SERIAL.begin(115200);
}
