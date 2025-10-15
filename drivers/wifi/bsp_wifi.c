#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "wifi_device.h"
#include "wifi_hotspot.h"
#include "lwip/netifapi.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/api_shell.h"
#include "bsp_wifi.h"



//WIFI通道
#define WIFI_CHANNEL    5

#define DEF_TIMEOUT 15

#define SELECT_WLAN_PORT "wlan0"

//STA 连接状态结果
int g_ConnectState = 0;

struct netif *g_lwip_netif = NULL;

//----------------------------WIFI AP----------------------------------
/** Hotspot state change */
void OnHotspotStateChangedCallbak(int state)
{
    printf("OnHotspotStateChangedCallbak: state is %d.\n", state);
    if (WIFI_HOTSPOT_ACTIVE == state)
    {
        printf("wifi hotspot active\n");
    }
    else
    {
        printf("wifi hotspot noactive\n");
    }
}

/** Station connected */
void OnHotspotStaJoinCallbak(StationInfo *info)
{
    static char macAddr[32] = {0};
    static unsigned char *mac = NULL;

    if (NULL == info)
    {
        printf("OnHotspotStaJoinCallbak is NULL\n");
    }
    else
    {
        mac = info->macAddress;
        snprintf(macAddr, sizeof(macAddr), "%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        printf("OnHotspotStaJoinCallbak: mac: %s reason: %d\n", macAddr, info->disconnectedReason);
    }
}

/** Station disconnected */
void OnHotspotStaLeaveCallbak(StationInfo *info)
{
    static char macAddr[32] = {0};
    static unsigned char *mac = NULL;

    if (NULL == info)
    {
        printf("OnHotspotStaLeaveCallbak is NULL\n");
    }
    else
    {
        mac = info->macAddress;
        snprintf(macAddr, sizeof(macAddr), "%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        printf("OnHotspotStaLeaveCallbak: mac: %s reason: %d\n", macAddr, info->disconnectedReason);
    }

}

//创建Wifi热点
WifiErrorCode WiFi_createHotSpots(const char *ssid, const char *psk)
{
    WifiErrorCode ret;

    static WifiEvent event;

    static HotspotConfig config;

    printf("Start Initialization of WiFI AP Mode\r\n");

    //注册WIFI事件的回调函数
    event.OnHotspotStaJoin = OnHotspotStaJoinCallbak;
    event.OnHotspotStaLeave = OnHotspotStaLeaveCallbak;
    event.OnHotspotStateChanged =OnHotspotStateChangedCallbak;
    ret = RegisterWifiEvent(&event);
    if (WIFI_SUCCESS != ret)
    {
        printf("RegisterWifiEvent failed....\n");
        return -1;
    }
    printf("RegisterWifiEvent OK .....\n");

    //设置热点
    strcpy(config.ssid, ssid);
    strcpy(config.preSharedKey, psk);
    config.band = HOTSPOT_BAND_TYPE_2G;
    config.channelNum = WIFI_CHANNEL;
    config.securityType = WIFI_SEC_TYPE_PSK;
    ret = SetHotspotConfig(&config);
    if (WIFI_SUCCESS != ret)
    {
        printf("SetHotspotConfig failed....\n");
        return -1;
    }
    printf("SetHotspotConfig OK....\n");

    //启动WIFI AP模式
    ret = EnableHotspot();
    if (WIFI_SUCCESS != ret)
    {
        printf("EnableHotspot failed...\n");
        return -1;
    }
    printf("EnableHotspot OK ....\n");

    //检查热点模式是否使能
    if (WIFI_HOTSPOT_ACTIVE != IsHotspotActive())
    {
        printf("IsHotspotActive failed....\n");
        return -1;
    }
    printf("IsHotspotActive OK .....\n");
}


//----------------------------WIFI STA----------------------------------


/** Connection state change */
void staOnWifiConnectionChanged(int state, WifiLinkedInfo *info)
{
    if (state > 0)
    {
        g_ConnectState = 1;
        printf("staOnWifiConnectionChanged state: %d\n", state);
    }
    else
    {
        printf("staOnWifiConnectionChanged failed state: %d\n", state);
    }
    
}

/** Scan state change */
void staOnWifiScanStateChanged(int state, int size)
{
    printf("staOnWifiScanStateChanged state: %d size: %d\n", state, size);
}

/** Hotspot state change */
void staOnHotspotStateChanged(int state)
{
    printf("staOnHotspotStateChanged state: %d\n", state);
}

/** Station connected */
void staOnHotspotStaJoin(StationInfo *info)
{
    printf("staOnHotspotStaJoin STA Join AP\n");
}

/** Station disconnected */
void staOnHotspotStaLeave(StationInfo *info)
{
    printf("staOnHotspotStaLeave..\n");
}


//STA模式 连接WIFI
WifiErrorCode WiFi_connectHotspots(const char *ssid, const char *psk)
{
    WifiErrorCode ret;

    static WifiEvent event;
    static WifiDeviceConfig config;
    int result;
    int timeout;

    printf("---------------WIFI STA Mode------------\n");
    //1. 注册WIFI事件
    event.OnHotspotStaJoin = staOnHotspotStaJoin;
    event.OnHotspotStaLeave = staOnHotspotStaLeave;
    event.OnHotspotStateChanged = staOnWifiScanStateChanged;
    event.OnWifiConnectionChanged = staOnWifiConnectionChanged;
    event.OnWifiScanStateChanged = staOnWifiScanStateChanged;
    ret = RegisterWifiEvent(&event);
    if (WIFI_SUCCESS != ret)
    {
        printf("RegisterWifiEvent failed....\n");
        return -1;
    }
    else
    {
        printf("RegisterWifiEvent OK....\n");
    }

    //2. 使能WIFI
    ret = EnableWifi();
    if (WIFI_SUCCESS != ret)
    {
        printf("EnableWifi failed...\n");
        return -1;
    }

    //3. 判断WIFI是否激活
    if (WIFI_STA_ACTIVE != IsWifiActive())
    {
        printf("IsWifiActive is not actived..\n");
        return -1;
    }

    //4. 配置连接热点信息
    strcpy(config.ssid, ssid);
    strcpy(config.preSharedKey, psk);
    config.securityType = WIFI_SEC_TYPE_PSK;
    ret = AddDeviceConfig(&config, &result);
    if (WIFI_SUCCESS != ret)
    {
        printf("AddDeviceConfig failed....\n");
        return -1;
    }

    //5. 连接到热点
    if (WIFI_SUCCESS == ConnectTo(result))
    {
        printf("ConnectTo OK.....\n");
    }
    else
    {
        printf("ConnectTo failed....\n");
        return -1;
    }

    //6. 等待连接结果
    timeout = DEF_TIMEOUT;
    while(timeout > 0)
    {
        sleep(1);
        timeout--;
        if (1 == g_ConnectState)
        {
            printf("Connect to %s OK ....\n", ssid);
            break;
        }
    }
    if (timeout <= 0)
    {
        printf("Connect to %s timeout.....\n", ssid);
        return -1;
    }

    //7. 获取网络接口
    g_lwip_netif = netifapi_netif_find(SELECT_WLAN_PORT);


    //8. 启动DHCP
    if (NULL != g_lwip_netif)
    {
        dhcp_start(g_lwip_netif);
        printf("dhcp_start begin dhcp....\n");
    }

    //9. 等待DHCP
    for (;;)
    {
        if (dhcp_is_bound(g_lwip_netif) == ERR_OK)
        {
            //打印获取到的IP信息
            netifapi_netif_common(g_lwip_netif, dhcp_clients_info_show, NULL);
            break;
        }

        printf("DHCP IP InProgress.....\n");
        sleep(1);
    }

    //10. 执行其它操作

}


