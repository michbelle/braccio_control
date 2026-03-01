#ifndef MAIN_WIFI_ZENESP
#define MAIN_WIFI_ZENESP

#ifdef __cplusplus
extern "C" {
#endif

#include <secrets.h>

#include <esp_wifi.h>
#include <nvs_flash.h>



void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
// static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

int init_communication_wifi(void);


#ifdef __cplusplus
}
#endif

#endif /* MAIN_WIFI_ZENESP */
