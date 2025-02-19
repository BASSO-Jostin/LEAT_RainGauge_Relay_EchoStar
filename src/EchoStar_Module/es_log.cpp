#include "es_log.h"
#include "es_delay.h"
#include "project_configuration.h"
#include <SD.h>

// TODO: Implement the SD Card logging feature

es_log::es_log()
{

#if defined(USING_LOG_USB_SERIAL)
#endif /* USING_LOG_USB_SERIAL */
}

es_log::~es_log()
{

#if defined(USING_LOG_USB_SERIAL)
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::init(void)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.begin(115200);
    while ((!USB_SERIAL) && (millis() < 5000))
        ;

    DELAY_MANAGER.delay_ms(100);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(void)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println();
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(String s)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(s);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(String s)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(s);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(const char *s)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(s);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(const char *s)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(s);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(char c)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(c);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(char c)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(c);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(int i)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(i);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print_hex(int i)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(i, HEX);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(unsigned int i)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(i);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(unsigned int i)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(i);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(int i)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(i);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(float f)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(f);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(float f)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(f);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::print(double d)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.print(d);
#endif /* USING_LOG_USB_SERIAL */
}

void es_log::println(double d)
{

#if defined(USING_LOG_USB_SERIAL)
    USB_SERIAL.println(d);
#endif /* USING_LOG_USB_SERIAL */
}

es_log LOG;