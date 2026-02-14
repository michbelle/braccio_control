#ifndef MAIN_WIFI_ZENESP
#define MAIN_WIFI_ZENESP

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_wifi.h>
#include <secrets.h>


static bool comm_is_connected = false;
static EventGroupHandle_t s_event_group_handler;
static int s_retry_count = 0;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

void init_communication(void);


#ifdef __cplusplus
}
#endif

#endif /* MAIN_WIFI_ZENESP */
