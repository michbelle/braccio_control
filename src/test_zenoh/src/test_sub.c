//
// Copyright (c) 2022 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//

#include <ctype.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <zenoh-pico.h>

#if Z_FEATURE_SUBSCRIPTION == 1

static int msg_nb = 0;

static int parse_args(int argc, char **argv, z_owned_config_t *config, char **keyexpr, int *n);

typedef struct position_motor{
    float rot1;
    float arm1_up;
    float arm2_up;
    float arm3_up;
    float rot_grasp;
    float grasp;
} position_motor;


position_motor ctrl_position_motor;


void data_handler(z_loaned_sample_t *sample, void *ctx) {
    (void)(ctx);
    printf("data:\n");
    z_view_string_t keystr;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &keystr);
    const z_loaned_bytes_t * payload = z_sample_payload(sample);
    // float ctrl_position_motor;
    // ze_deserialize_float(payload, &ctrl_position_motor);
    // printf("%f\n",ctrl_position_motor);
    ze_deserializer_t data = ze_deserializer_from_bytes(payload);
    ze_deserializer_deserialize_float(&data, &ctrl_position_motor.rot1);
    printf("%f\n",ctrl_position_motor.rot1);
    ze_deserializer_deserialize_float(&data, &ctrl_position_motor.arm1_up);
    printf("%f\n",ctrl_position_motor.arm1_up);
    ze_deserializer_deserialize_float(&data, &ctrl_position_motor.arm2_up);
    printf("%f\n",ctrl_position_motor.arm2_up);
    ze_deserializer_deserialize_float(&data, &ctrl_position_motor.arm3_up);
    printf("%f\n",ctrl_position_motor.arm3_up);
    ze_deserializer_deserialize_float(&data, &ctrl_position_motor.rot_grasp);
    printf("%f\n",ctrl_position_motor.rot_grasp);
    ze_deserializer_deserialize_float(&data, &ctrl_position_motor.grasp);
    printf("%f\n\n",ctrl_position_motor.grasp);
}

int main(int argc, char **argv) {
    char *keyexpr = "braccio_control/positions";
    int n = 0;

    z_owned_config_t config;
    z_config_default(&config);
    // zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, "peer");
    // zp_config_insert(z_loan_mut(config), Z_CONFIG_LISTEN_KEY, "tcp/224.0.0.225:7447");

    int ret = parse_args(argc, argv, &config, &keyexpr, &n);
    if (ret != 0) {
        return ret;
    }

    printf("Opening session...\n");
    z_owned_session_t s;
    if (z_open(&s, z_move(config), NULL) < 0) {
        printf("Unable to open session!\n");
        return -1;
    }

    // Start read and lease tasks for zenoh-pico
    if (zp_start_read_task(z_loan_mut(s), NULL) < 0 || zp_start_lease_task(z_loan_mut(s), NULL) < 0) {
        printf("Unable to start read and lease tasks\n");
        z_session_drop(z_session_move(&s));
        return -1;
    }

#if Z_FEATURE_ADMIN_SPACE == 1
    // Start admin space
    if (zp_start_admin_space(z_loan_mut(s)) < 0) {
        printf("Unable to start admin space\n");
        z_drop(z_move(s));
        return -1;
    }
#endif

    z_owned_closure_sample_t callback;
    z_closure(&callback, data_handler, NULL, NULL);
    printf("Declaring Subscriber on '%s'...\n", keyexpr);
    z_owned_subscriber_t sub;
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
        printf("%s is not a valid key expression\n", keyexpr);
        return -1;
    }
    if (z_declare_subscriber(z_loan(s), &sub, z_loan(ke), z_move(callback), NULL) < 0) {
        printf("Unable to declare subscriber.\n");
        return -1;
    }

    printf("Press CTRL-C to quit...\n");
    while (1) {
        if ((n != 0) && (msg_nb > n)) {
            break;
        }
        sleep(1);
    }
    // Clean up
    z_drop(z_move(sub));
    printf("drop sub\n");
    z_drop(z_move(s));
    printf("drop s\n");
    return 0;
}

// Note: All args can be specified multiple times. For "-e" it will append the list of endpoints, for the other it will
// simply replace the previous value.
static int parse_args(int argc, char **argv, z_owned_config_t *config, char **ke, int *n) {
    int opt;
    while ((opt = getopt(argc, argv, "k:e:m:l:n:")) != -1) {
        switch (opt) {
            case 'k':
                *ke = optarg;
                break;
            case 'e':
                zp_config_insert(z_loan_mut(*config), Z_CONFIG_CONNECT_KEY, optarg);
                break;
            case 'm':
                zp_config_insert(z_loan_mut(*config), Z_CONFIG_MODE_KEY, optarg);
                break;
            case 'l':
                zp_config_insert(z_loan_mut(*config), Z_CONFIG_LISTEN_KEY, optarg);
                break;
            case 'n':
                *n = atoi(optarg);
                break;
            case '?':
                if (optopt == 'k' || optopt == 'e' || optopt == 'm' || optopt == 'l' || optopt == 'n') {
                    fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                } else {
                    fprintf(stderr, "Unknown option `-%c'.\n", optopt);
                }
                return 1;
            default:
                return -1;
        }
    }
    return 0;
}

#else
int main(void) {
    printf("ERROR: Zenoh pico was compiled without Z_FEATURE_SUBSCRIPTION but this example requires it.\n");
    return -2;
}
#endif
