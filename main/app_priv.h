/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include <esp_err.h>
#include <esp_matter.h>
#include "driver/gpio.h"  // Required for gpio_num_t


#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include "esp_openthread_types.h"
#endif

#define SERVO_MIN_DUTY 500  // Example: 0° position
#define SERVO_MAX_DUTY 2500 // Example: 180° position

typedef void *app_driver_handle_t;

/** Initialize the autofeeder driver
 *
 * This initializes the light driver associated with the selected board.
 *
 * @return Handle on success.
 * @return NULL in case of failure.
 */
app_driver_handle_t app_driver_autofeeder_init();

/** Initialize the servo driver
 *
 * This initializes the servo driver associated with the selected board.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t app_driver_servo_init(gpio_num_t gpio);

/** Map speed to servo angle
 *
 * This API maps the fan speed to the servo angle.
 *
 * @param[in] speed Fan speed.
 *
 * @return Servo angle.
 */
int map_speed_to_servo_angle(uint8_t speed);

/** Set servo angle
 *
 * This API sets the servo angle based on the fan speed.
 *
 * @param[in] angle Servo angle.
 *
 * @return ESP_OK on success.
 */
esp_err_t set_servo_angle(uint8_t angle);

/** Driver Update
 *
 * This API should be called to update the driver for the attribute being updated.
 * This is usually called from the common `app_attribute_update_cb()`.
 *
 * @param[in] endpoint_id Endpoint ID of the attribute.
 * @param[in] cluster_id Cluster ID of the attribute.
 * @param[in] attribute_id Attribute ID of the attribute.
 * @param[in] val Pointer to `esp_matter_attr_val_t`. Use appropriate elements as per the value type.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val);

/** Initialize the driver components
 *
 * This API initializes the driver components.
 *
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t app_driver_servo_set_defaults(uint16_t endpoint_id);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()                                           \
    {                                                                                   \
        .radio_mode = RADIO_MODE_NATIVE,                                                \
    }

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()                                            \
    {                                                                                   \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE,                              \
    }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                            \
    {                                                                                   \
        .storage_partition_name = "nvs", .netif_queue_size = 10, .task_queue_size = 10, \
    }
#endif
