#ifndef __BSP_WIFI_H__
#define __BSP_WIFI_H__


#include "cmsis_os2.h"
#include "hi_io.h"
#include "hi_gpio.h"
#include "wifi_error_code.h"
#include "wifi_device.h"

//函数声明
WifiErrorCode WiFi_createHotSpots(const char *ssid, const char *psk);

WifiErrorCode WiFi_connectHotspots(const char *ssid, const char *psk);

#endif /* __BSP_WIFI_H__ */


