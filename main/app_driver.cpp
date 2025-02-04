#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_matter.h"
#include "app_priv.h"

#include <inttypes.h>

using namespace chip::app::Clusters;
using namespace esp_matter;

static const char *TAG = "app_driver";
extern uint16_t light_endpoint_id;

// Define MCPWM components
static mcpwm_oper_handle_t mcpwm_oper = NULL;
static mcpwm_cmpr_handle_t comparator = NULL;
static mcpwm_gen_handle_t generator = NULL;
static mcpwm_timer_handle_t timer = NULL;

// Global variable to store the last commanded angle (in degrees)
static int last_angle = 0;

// Define servo parameters for an SG90 (pulse widths corresponding to -90° to +90°)
#define SERVO_MIN_PULSEWIDTH_US 1000  // Pulse width for -90°
#define SERVO_MAX_PULSEWIDTH_US 2000  // Pulse width for +90°
#define SERVO_MIN_DEGREE        -90    // Minimum angle in degrees
#define SERVO_MAX_DEGREE         90    // Maximum angle in degrees

#define SERVO_PERIOD 20000   // 20ms period (50 Hz)

/**
 * @brief Convert a servo angle (in degrees, -90 to +90) to a PWM compare value (in µs)
 */
static inline uint32_t example_angle_to_compare(int angle)
{
    return (uint32_t)(((angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)) /
                      (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US);
}

/**
 * @brief Initialize the servo motor on a given GPIO pin.
 */
esp_err_t app_driver_servo_init(gpio_num_t gpio) {
    ESP_LOGI(TAG, "Initializing servo on GPIO %d", gpio);

    // Timer Configuration (using group_id = 1 to avoid conflicts)
    mcpwm_timer_config_t timer_config = {
        .group_id = 1,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,  // 1MHz => 1 µs per tick
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_PERIOD,  // 20ms period
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // Operator Configuration (using group_id = 1)
    mcpwm_operator_config_t oper_config = {
        .group_id = 1
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &mcpwm_oper));

    // Connect Timer to Operator
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(mcpwm_oper, timer));

    // Removed mcpwm_timer_start(timer) because it's not needed in this API version

    // Comparator Configuration
    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(mcpwm_oper, &comparator_config, &comparator));

    // Generator Configuration
    mcpwm_generator_config_t generator_config = {};
    generator_config.gen_gpio_num = gpio;
    generator_config.flags.invert_pwm = false;
    ESP_ERROR_CHECK(mcpwm_new_generator(mcpwm_oper, &generator_config, &generator));

    // Set initial servo position to 0° (center)
    last_angle = 0;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));
    ESP_LOGI(TAG, "Servo initialized on GPIO %d", gpio);

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    return ESP_OK;
}

/**
 * @brief Map fan speed (0-100) to a servo angle (-90° to +90°).
 *
 * Converts a percentage speed value to an angle where:
 *   0% → -90°, 50% → 0°, and 100% → +90°.
 */
int map_speed_to_servo_angle(uint8_t speed) {
    return ((int)speed * 180) / 100 - 90;
}

/**
 * @brief Set the servo angle (in the range -90° to +90°) and log the corresponding PWM value.
 */
esp_err_t set_servo_angle(int angle) {
    last_angle = angle; // Update global last angle
    uint32_t duty = example_angle_to_compare(angle);
    ESP_LOGI(TAG, "Setting servo angle: %d° -> PWM: %" PRIu32 "us", angle, duty);

    esp_err_t err = mcpwm_comparator_set_compare_value(comparator, duty);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set servo PWM value");
    }
    return err;
}

/**
 * @brief Update attributes from Matter and apply changes.
 */
esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id,
                                       uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val) {
    ESP_LOGI(TAG, "Attribute update: Cluster ID: %lu, Attribute ID: %lu", cluster_id, attribute_id);

    if (cluster_id == FanControl::Id) {
        if (attribute_id == FanControl::Attributes::PercentSetting::Id) {
            uint8_t speed = val->val.u8;
            int angle = map_speed_to_servo_angle(speed);
            return set_servo_angle(angle);
        }
    }
    return ESP_OK;
}

/**
 * @brief Initialize the driver components and set default servo values.
 */
esp_err_t app_driver_servo_set_defaults(uint16_t endpoint_id)
{
    esp_err_t err = ESP_OK;

    // Retrieve private data assigned to this endpoint (should contain servo GPIO)
    void *priv_data = endpoint::get_priv_data(endpoint_id);
    gpio_num_t servo_gpio = (gpio_num_t)(uintptr_t)priv_data;  // Convert to GPIO number

    ESP_LOGI(TAG, "Setting default servo values for endpoint: %d on GPIO %d", endpoint_id, servo_gpio);

    /* Initialize Servo */
    err |= app_driver_servo_init(servo_gpio);

    /* Set default position based on attribute PercentSetting.
       Here, we assume that a speed of 50% corresponds to a 0° (center) position. */
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute_t *attribute = attribute::get(endpoint_id, FanControl::Id, FanControl::Attributes::PercentSetting::Id);

    if (attribute) {
        esp_matter::attribute::get_val(attribute, &val);
        // If the received value is out of the expected 0-100 range, default to 50%
        if (val.val.u8 > 100) {  
            val.val.u8 = 50;
            ESP_LOGI(TAG, "Setting default servo position to 0° (50%% speed)");
        }
    } else {
        ESP_LOGW(TAG, "Attribute PercentSetting not found, setting to default (0°)");
        val.val.u8 = 50;
    }

    /* Apply default servo position */
    err |= set_servo_angle(map_speed_to_servo_angle(val.val.u8));

    return err;
}

/**
 * @brief Initialize the autofeeder (servo motor) and return its handle.
 */
app_driver_handle_t app_driver_autofeeder_init()
{
    ESP_LOGI(TAG, "Initializing Autofeeder (Servo Motor) on GPIO 21");
    gpio_num_t servo_gpio = GPIO_NUM_21;

    if (app_driver_servo_init(servo_gpio) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize servo motor!");
        return NULL;
    }

    ESP_LOGI(TAG, "Autofeeder initialized successfully!");
    return (app_driver_handle_t)(uintptr_t)servo_gpio;  // Return GPIO number as handle
}
