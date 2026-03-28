#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/split_peripheral_status_changed.h>
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/split/bluetooth/central.h>
#endif
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <zmk/battery.h>

LOG_MODULE_REGISTER(status_leds, CONFIG_ZMK_LOG_LEVEL);

#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static const struct gpio_dt_spec left_leds[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_caps_lock), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_num_lock), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_scroll_lock), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_left_battery_full), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_left_low_battery), gpios),
};
#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static const struct gpio_dt_spec right_leds[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_layer1), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_layer2), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_layer3), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_split_disconnect), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_bt_disconnect), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_bt_pairing), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_right_battery_full), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_right_low_battery), gpios),
};
#endif

// LED indices
enum left_led_index {
    LEFT_LED_CAPS = 0,
    LEFT_LED_NUM,
    LEFT_LED_SCROLL,
    LEFT_LED_BATTERY_FULL,
    LEFT_LED_LOW_BATTERY,
};

enum right_led_index {
    RIGHT_LED_LAYER1 = 0,
    RIGHT_LED_LAYER2,
    RIGHT_LED_LAYER3,
    RIGHT_LED_SPLIT_DISCONN,
    RIGHT_LED_BT_DISCONN,
    RIGHT_LED_PAIRING,
    RIGHT_LED_BATTERY_FULL,
    RIGHT_LED_LOW_BATTERY,
};

static void set_led(const struct gpio_dt_spec *led, bool state) {
    if (!device_is_ready(led->port)) {
        return;
    }
    gpio_pin_set_dt(led, state ? 1 : 0);
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#define HIGHEST_LAYER (ZMK_KEYMAP_LAYERS_LEN - 1)
static uint8_t get_highest_layer(zmk_keymap_layers_state_t state) {
    // Find the highest active layer from the bitmask
    for (int8_t current_layer = HIGHEST_LAYER; current_layer >= 0; current_layer--) {
        if (state & BIT(current_layer)) {
            return current_layer;
        }
    }
    return 0; // Default to layer 0
}

static void update_layer_leds() {
    uint8_t current_layer = get_highest_layer(zmk_keymap_layer_state());
    set_led(&right_leds[RIGHT_LED_LAYER1], current_layer == 1);
    set_led(&right_leds[RIGHT_LED_LAYER2], current_layer == 2);
    set_led(&right_leds[RIGHT_LED_LAYER3], current_layer == 3);
}
#endif

//HID indicators are handled directly by ZMK, don't worry about them here

#define LOW_BATTERY_THRESHOLD 15
#define BATTERY_FULL 100

static void update_battery_leds() {
    uint8_t level = zmk_battery_state_of_charge();
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    // Right half
    set_led(&right_leds[RIGHT_LED_BATTERY_FULL], level == BATTERY_FULL);
    set_led(&right_leds[RIGHT_LED_LOW_BATTERY], level <= LOW_BATTERY_THRESHOLD);
#else
    set_led(&left_leds[LEFT_LED_BATTERY_FULL], level == BATTERY_FULL);
    set_led(&left_leds[LEFT_LED_LOW_BATTERY], level <= LOW_BATTERY_THRESHOLD);
#endif
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
//the only one of these that works is the main connection status; I need to fix the other two
static void update_main_connection_status(void) {
    // Check bluetooth host connection
    struct zmk_endpoint_instance endpoint = zmk_endpoints_selected();
    bool bt_connected = (endpoint.transport == ZMK_TRANSPORT_BLE) && 
                        zmk_ble_active_profile_is_connected();
    
    set_led(&right_leds[RIGHT_LED_BT_DISCONN], !bt_connected);
    
    // Pairing mode (profile is open but not connected)
    bool pairing = (endpoint.transport == ZMK_TRANSPORT_BLE) && 
                       zmk_ble_active_profile_is_open() && !bt_connected;
    set_led(&right_leds[RIGHT_LED_PAIRING], pairing);
}

static void update_peripheral_connection_status(bool peripheral_connected) {
    set_led(&right_leds[RIGHT_LED_SPLIT_DISCONN], !peripheral_connected);
}
#endif

// Event handlers
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static int status_led_layer_event_listener(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    
    update_layer_leds();
    return ZMK_EV_EVENT_BUBBLE;
}
#endif

static int status_led_battery_event_listener(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    
    update_battery_leds();
    return ZMK_EV_EVENT_BUBBLE;
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static int status_led_split_event_listener(const zmk_event_t *eh) {
    const struct zmk_split_peripheral_status_changed *ev = 
        as_zmk_split_peripheral_status_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    
    update_peripheral_connection_status(ev->connected);
    return ZMK_EV_EVENT_BUBBLE;
}
#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static int status_led_ble_event_listener(const zmk_event_t *eh) {
    update_main_connection_status();
    return ZMK_EV_EVENT_BUBBLE;
}
#endif

// Register event listeners
ZMK_LISTENER(status_leds_battery, status_led_battery_event_listener);
ZMK_SUBSCRIPTION(status_leds_battery, zmk_battery_state_changed);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
ZMK_LISTENER(status_leds_layer, status_led_layer_event_listener);
ZMK_SUBSCRIPTION(status_leds_layer, zmk_layer_state_changed);

ZMK_LISTENER(status_leds_split, status_led_split_event_listener);
ZMK_SUBSCRIPTION(status_leds_split, zmk_split_peripheral_status_changed);

ZMK_LISTENER(status_leds_ble, status_led_ble_event_listener);
ZMK_SUBSCRIPTION(status_leds_ble, zmk_ble_active_profile_changed);
#endif

static int status_leds_init(void) {
    int ret = 0;

    #if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    // Initialize right LEDs
    for (int i = 0; i < ARRAY_SIZE(right_leds); i++) {
        if (!device_is_ready(right_leds[i].port)) {
            LOG_ERR("Right LED %d device not ready", i);
            continue;
        }
        ret = gpio_pin_configure_dt(&right_leds[i], GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure right LED %d: %d", i, ret);
        }
    }
    #else
    // Initialize left LEDs
    for (int i = 0; i < ARRAY_SIZE(left_leds); i++) {
        if (!device_is_ready(left_leds[i].port)) {
            LOG_ERR("Left LED %d device not ready", i);
            continue;
        }
        ret = gpio_pin_configure_dt(&left_leds[i], GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure left LED %d: %d", i, ret);
        }
    }
    #endif
    
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    update_layer_leds();
    update_main_connection_status();
    update_peripheral_connection_status(false);
#endif
    
    // Get initial battery state
    update_battery_leds();
    
    LOG_INF("Status LEDs initialized");
    return 0;
}

SYS_INIT(status_leds_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
