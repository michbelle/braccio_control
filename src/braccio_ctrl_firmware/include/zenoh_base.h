#ifndef MAIN_ZENOH_CONFIG
#define MAIN_ZENOH_CONFIG

#ifdef __cplusplus
extern "C" {
#endif

#include <zenoh-pico.h>


void init_zenoh();
z_owned_session_t s;


#ifdef __cplusplus
}
#endif

#endif /* MAIN_ZENOH_CONFIG */