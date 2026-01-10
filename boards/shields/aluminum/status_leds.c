#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/split_peripheral_status_changed.h>
#include <zmk/hid_indicators_changed.h>
#include <zmk/split/bluetooth/central.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>

#define ZMK_HID_INDICATOR_NUM_LOCK    (1 << 0)  // Bit 0
#define ZMK_HID_INDICATOR_CAPS_LOCK   (1 << 1)  // Bit 1
#define ZMK_HID_INDICATOR_SCROLL_LOCK (1 << 2)  // Bit 2

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// LED definitions for left half (peripheral) - lock indicators
#define LEFT_CAPS_LED_NODE DT_ALIAS(led_caps_lock)
#define LEFT_NUM_LED_NODE DT_ALIAS(led_num_lock)
#define LEFT_SCROLL_LED_NODE DT_ALIAS(led_scroll_lock)
#define LEFT_CHARGING_LED_NODE DT_ALIAS(led_left_charging)
#define LEFT_LOW_BATTERY_LED_NODE DT_ALIAS(led_left_low_battery)

// LED definitions for right half (central/master) - layer indicators
#define RIGHT_LAYER1_LED_NODE DT_ALIAS(led_layer1)
#define RIGHT_LAYER2_LED_NODE DT_ALIAS(led_layer2)
#define RIGHT_LAYER3_LED_NODE DT_ALIAS(led_layer3)
#define RIGHT_SPLIT_DISCONN_LED_NODE DT_ALIAS(led_split_disconnect)
#define RIGHT_BT_DISCONN_LED_NODE DT_ALIAS(led_bt_disconnect)
#define RIGHT_PAIRING_LED_NODE DT_ALIAS(led_bt_pairing)
#define RIGHT_CHARGING_LED_NODE DT_ALIAS(led_right_charging)
#define RIGHT_LOW_BATTERY_LED_NODE DT_ALIAS(led_right_low_battery)

// GPIO specs
static const struct gpio_dt_spec left_leds[] = {
#if DT_NODE_HAS_STATUS(LEFT_CAPS_LED_NODE, okay)
    GPIO_DT_SPEC_GET(LEFT_CAPS_LED_NODE, gpios),
#endif
#if DT_NODE_HAS_STATUS(LEFT_NUM_LED_NODE, okay)
    GPIO_DT_SPEC_GET(LEFT_NUM_LED_NODE, gpios),
#endif
#if DT_NODE_HAS_STATUS(LEFT_SCROLL_LED_NODE, okay)
    GPIO_DT_SPEC_GET(LEFT_SCROLL_LED_NODE, gpios),
#endif
#if DT_NODE_HAS_STATUS(LEFT_CHARGING_LED_NODE, okay)
    GPIO_DT_SPEC_GET(LEFT_CHARGING_LED_NODE, gpios),
#endif
#if DT_NODE_HAS_STATUS(LEFT_LOW_BATTERY_LED_NODE, okay)
    GPIO_DT_SPEC_GET(LEFT_LOW_BATTERY_LED_NODE, gpios),
#endif
};

static const struct gpio_dt_spec right_leds[] = {
#if DT_NODE_HAS_STATUS(RIGHT_LAYER1_LED_NODE, okay)
    GPIO_DT_SPEC_GET(RIGHT_LAYER1_LED_NODE, gpios),
#endif
#if DT_NODE_HAS_STATUS(RIGHT_LAYER2_LED_NODE, okay)
    GPIO_DT_SPEC_GET(RIGHT_LAYER2_LED_NODE, gpios),
#endif
#if DT_NODE_HAS_STATUS(RIGHT_LAYER3_LED_NODE, okay)
    GPIO_DT_SPEC_GET(RIGHT_LAYER3_LED_NODE, gpios),
#endif
#if DT_NODE_HAS_STATUS(RIGHT_SPLIT_DISCONN_LED_NODE, okay)
    GPIO_DT_SPEC_GET(RIGHT_SPLIT_DISCONN_LED_NODE, gpios),
#endif
#if DT_NODE_HAS_STATUS(RIGHT_BT_DISCONN_LED_NODE, okay)
    GPIO_DT_SPEC_GET(RIGHT_BT_DISCONN_LED_NODE, gpios),
#endif
#if DT_NODE_HAS_STATUS(RIGHT_PAIRING_LED_NODE, okay)
    GPIO_DT_SPEC_GET(RIGHT_PAIRING_LED_NODE, gpios),
#endif
#if DT_NODE_HAS_STATUS(RIGHT_CHARGING_LED_NODE, okay)
    GPIO_DT_SPEC_GET(RIGHT_CHARGING_LED_NODE, gpios),
#endif
#if DT_NODE_HAS_STATUS(RIGHT_LOW_BATTERY_LED_NODE, okay)
    GPIO_DT_SPEC_GET(RIGHT_LOW_BATTERY_LED_NODE, gpios),
#endif
};

// LED indices
enum left_led_index {
    LEFT_LED_CAPS = 0,
    LEFT_LED_NUM,
    LEFT_LED_SCROLL,
    LEFT_LED_CHARGING,
    LEFT_LED_LOW_BATTERY,
};

enum right_led_index {
    RIGHT_LED_LAYER1 = 0,
    RIGHT_LED_LAYER2,
    RIGHT_LED_LAYER3,
    RIGHT_LED_SPLIT_DISCONN,
    RIGHT_LED_BT_DISCONN,
    RIGHT_LED_PAIRING,
    RIGHT_LED_CHARGING,
    RIGHT_LED_LOW_BATTERY,
};

#define LOW_BATTERY_THRESHOLD 15
#define HIGHEST_LAYER 3

static void set_led(const struct gpio_dt_spec *led, bool state) {
    if (!device_is_ready(led->port)) {
        return;
    }
    gpio_pin_set_dt(led, state ? 1 : 0);
}

static uint8_t get_highest_layer(zmk_keymap_layers_state_t state) {
    // Find the highest active layer from the bitmask
    for (int8_t layer = HIGHEST_LAYER; layer >= 0; layer--) {
        if (state & BIT(layer)) {
            return layer;
        }
    }
    return 0; // Default to layer 0
}

static void update_layer_leds(uint8_t layer) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    uint8_t layer = get_highest_layer(state);
    // We're on the central (right), update layer LEDs
    set_led(&right_leds[RIGHT_LED_LAYER1], layer == 1);
    set_led(&right_leds[RIGHT_LED_LAYER2], layer == 2);
    set_led(&right_leds[RIGHT_LED_LAYER3], layer == 3);
#endif
}

static void update_hid_indicators(zmk_hid_indicators_t indicators) {
#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    // HID indicators on left (peripheral)
    set_led(&left_leds[LEFT_LED_CAPS], indicators & ZMK_HID_INDICATOR_CAPS_LOCK);
    set_led(&left_leds[LEFT_LED_NUM], !(indicators & ZMK_HID_INDICATOR_NUM_LOCK));
    set_led(&left_leds[LEFT_LED_SCROLL], indicators & ZMK_HID_INDICATOR_SCROLL_LOCK);
#endif
}

static void update_battery_leds(uint8_t level) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    // Right half
    // can't actually sense charging
    // if (ARRAY_SIZE(right_leds) > RIGHT_LED_CHARGING) {
    //     set_led(&right_leds[RIGHT_LED_CHARGING], charging);
    // }
    if (ARRAY_SIZE(right_leds) > RIGHT_LED_LOW_BATTERY) {
        set_led(&right_leds[RIGHT_LED_LOW_BATTERY], level <= LOW_BATTERY_THRESHOLD);
    }
#else
    // Left half
    // if (ARRAY_SIZE(left_leds) > LEFT_LED_CHARGING) {
    //     set_led(&left_leds[LEFT_LED_CHARGING], charging);
    // }
    if (ARRAY_SIZE(left_leds) > LEFT_LED_LOW_BATTERY) {
        set_led(&left_leds[LEFT_LED_LOW_BATTERY], level <= LOW_BATTERY_THRESHOLD);
    }
#endif
}

static void update_connection_status(void) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    // Check split connection status
    bool split_connected = zmk_split_bt_peripheral_is_connected();
    set_led(&right_leds[RIGHT_LED_SPLIT_DISCONN], !split_connected);

    // Check bluetooth host connection
    struct zmk_endpoint_instance endpoint = zmk_endpoints_selected();
    bool bt_connected = (endpoint.transport == ZMK_TRANSPORT_BLE) && 
                        zmk_ble_active_profile_is_connected();
    
    set_led(&right_leds[RIGHT_LED_BT_DISCONN], !bt_connected);
    
    // Pairing mode (profile is open but not connected)
    bool pairing = (endpoint.transport == ZMK_TRANSPORT_BLE) && 
                       zmk_ble_active_profile_is_open() && !bt_connected;
    set_led(&right_leds[RIGHT_LED_PAIRING], pairing);
#endif
}

// Event handlers
static int status_led_layer_event_listener(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    
    update_layer_leds(zmk_keymap_highest_layer_active());
    return ZMK_EV_EVENT_BUBBLE;
}

static int status_led_battery_event_listener(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    
    update_battery_leds(ev->state_of_charge);
    return ZMK_EV_EVENT_BUBBLE;
}

static int status_led_hid_indicators_event_listener(const zmk_event_t *eh) {
#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    
    update_hid_indicators(ev->indicators);
#endif
    return ZMK_EV_EVENT_BUBBLE;
}

static int status_led_split_event_listener(const zmk_event_t *eh) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    const struct zmk_split_peripheral_status_changed *ev = 
        as_zmk_split_peripheral_status_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    
    update_connection_status();
#endif
    return ZMK_EV_EVENT_BUBBLE;
}

static int status_led_ble_event_listener(const zmk_event_t *eh) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    update_connection_status();
#endif
    return ZMK_EV_EVENT_BUBBLE;
}



// Indicator update callback
static void indicators_changed_callback(zmk_hid_indicators_t indicators) {
    update_hid_indicators(indicators);
}

// Register event listeners
ZMK_LISTENER(status_leds_layer, status_led_layer_event_listener);
ZMK_SUBSCRIPTION(status_leds_layer, zmk_layer_state_changed);

ZMK_LISTENER(status_leds_battery, status_led_battery_event_listener);
ZMK_SUBSCRIPTION(status_leds_battery, zmk_battery_state_changed);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
ZMK_LISTENER(status_leds_split, status_led_split_event_listener);
ZMK_SUBSCRIPTION(status_leds_split, zmk_split_peripheral_status_changed);

ZMK_LISTENER(status_leds_ble, status_led_ble_event_listener);
ZMK_SUBSCRIPTION(status_leds_ble, zmk_ble_active_profile_changed);
#endif

#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
ZMK_LISTENER(status_leds_hid_indicators, status_led_hid_indicators_event_listener);
ZMK_SUBSCRIPTION(status_leds_hid_indicators, zmk_hid_indicators_changed);
#endif

static int status_leds_init(void) {
    int ret;
    
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
     
    // Initial state update - delay to allow other subsystems to init
    k_msleep(100);
    
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    update_layer_leds(BIT(0));
    update_connection_status();
#else
    // Peripheral gets initial HID indicators
    update_hid_indicators(0);
#endif
    
    // Get initial battery state
#if IS_ENABLED(CONFIG_ZMK_BATTERY)
    uint8_t battery_level = zmk_battery_state_of_charge();
    update_battery_leds(battery_level);
#endif
    
    LOG_INF("Status LEDs initialized");
    return 0;
}

SYS_INIT(status_leds_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
