#include "es_log.h"
#include "es_delay.h"
#include "project_configuration.h"

#if defined(USING_LOG_SD_CARD)
#include <SD.h>
#endif /* USING_LOG_SD_CARD */

es_log::es_log()
{

#if defined(USING_LOG_SD_CARD)
    sd_available_flag = false;

#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
#endif /* USING_LOG_USB_SERIAL */
}

es_log::~es_log()
{

#if defined(USING_LOG_SD_CARD)
    // Close the file
    sd_card_close_log();

    // Set pinMode and Power OFF
    digitalWrite(SENSORS_PWR_ENABLE_PIN, LOW);
    pinMode(SENSORS_PWR_ENABLE_PIN, INPUT);

#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::init(void)
{

#if defined(USING_LOG_SD_CARD)

    // Set pinMode and Power ON
    pinMode(SENSORS_PWR_ENABLE_PIN, OUTPUT);
    digitalWrite(SENSORS_PWR_ENABLE_PIN, HIGH);

    // Init the SD card
    sd_available_flag = SD.begin(SDCARD_SS_PIN);
    if (sd_available_flag)
    {
        sd_available_flag = true;
        if (sd_card_open_log())
        {
            sd_log_file.println("\n==================================");
            sd_log_file.print(">> SD Card is successfully init at ");
            sd_log_file.println(millis());
            sd_log_file.println("==================================\n");

            sd_card_close_log();
        }
    }
    else
    {
        // Do Nothing

        // while (1) // Blocking program if no SDCard detected
        // {
        //     digitalWrite(LED_BUILTIN, HIGH);
        //     delay(50);
        //     digitalWrite(LED_BUILTIN, LOW);
        //     delay(150);
        //     digitalWrite(LED_BUILTIN, HIGH);
        //     delay(50);
        //     digitalWrite(LED_BUILTIN, LOW);
        //     delay(500 + 150);
        // }
    }

#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.begin(115200);
    while ((!USB_SERIAL) && (millis() < 5000))
        ;

    DELAY_MANAGER.delay_ms(100);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(void)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.println();
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println();
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(String s)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.print(s);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(s);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(String s)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.println(s);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(s);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(const char *s)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.print(s);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(s);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(const char *s)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.println(s);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(s);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(char c)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.print(c);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(c);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(char c)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.println(c);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(c);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(int i)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.print(i);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(i);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print_hex(int i)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.print(i, HEX);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(i, HEX);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(unsigned int i)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.println(i);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(i);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(unsigned int i)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.print(i);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(i);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(int i)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.println(i);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(i);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(float f)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.print(f);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(f);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(float f)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.println(f);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(f);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(double d)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.print(d);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(d);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(double d)
{

#if defined(USING_LOG_SD_CARD)
    if (sd_card_open_log())
    {
        sd_log_file.println(d);
        sd_card_close_log();
    }
#endif /* USING_LOG_SD_CARD */

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(d);
#endif /* USING_LOG_USB_SERIAL */
}

#if defined(USING_LOG_SD_CARD)

bool es_log::sd_card_open_log(void)
{
    if (sd_available_flag)
    {

        digitalWrite(SENSORS_PWR_ENABLE_PIN, HIGH);
        delay(5);

        sd_log_file = SD.open(SD_LOG_FILENAME, FILE_WRITE);
        return (bool)sd_log_file;
    }
    else
    {
        return false;
    }
}

void es_log::sd_card_close_log(void)
{
    if (sd_log_file)
    {
        sd_log_file.close();

        delay(5);
        digitalWrite(SENSORS_PWR_ENABLE_PIN, LOW);
    }
}

#endif /* USING_LOG_SD_CARD */

es_log LOG;
