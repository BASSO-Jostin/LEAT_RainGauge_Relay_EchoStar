#include "es_watchdog.h"

#include <IWatchdog.h>
#include <STM32RTC.h>

#ifdef USING_WATCHDOG

static STM32RTC &rtc = STM32RTC::getInstance();

es_watchdog::es_watchdog(void)
{
}

es_watchdog::~es_watchdog(void)
{
}

bool es_watchdog::isResetByWatchdog(void)
{
    bool reset_by_watchdog_flag = IWatchdog.isReset(true); // Clear the flag after read

    //LOG_D("[DEBUG] es_watchdog::isResetByWatchdog() | The last reset is caused by ");
    if (reset_by_watchdog_flag)
    {
        //LOG_D_LN("WATCHDOG");
    }
    else
    {
        //LOG_D_LN("EXTERNAL");
    }

    return reset_by_watchdog_flag;
}

bool es_watchdog::init(void)
{
    // LOG_D("[DEBUG] es_watchdog::init() | Init Watchdog with the timeout of ");
    // LOG_D((int) WATCHDOG_TIMEOUT_S);
    // LOG_Dln(" seconds");

    IWatchdog.begin(1000000 * WATCHDOG_TIMEOUT_S);
    return IWatchdog.isEnabled();
}

void es_watchdog::reload(void)
{
    // LOG_D("[DEBUG] es_watchdog::init() | A watchdog reload is called at ");
    // LOG_Dln((unsigned int)rtc.getEpoch());
    
    IWatchdog.reload();
}

#else /* USING_WATCHDOG is commented out / disable */

es_watchdog::es_watchdog(void)
{
}

es_watchdog::~es_watchdog(void)
{
}

bool es_watchdog::isResetByWatchdog(void)
{
    return false;
}

bool es_watchdog::init(void)
{
    return false;
}

void es_watchdog::reload(void)
{
}

#endif

es_watchdog WATCHDOG;