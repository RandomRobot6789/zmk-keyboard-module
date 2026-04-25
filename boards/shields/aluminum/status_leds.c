/*
 * status_leds.c — Custom LED status indicators for the Aluminum split keyboard.
 *
 * HID INDICATOR APPROACH
 * ----------------------
 * ZMK's built-in indicator_leds.c cannot compile on a split peripheral because
 * it calls zmk_endpoint_is_connected(), which is not compiled there (issue #3307).
 * Additionally, zmk_hid_indicators_get_current_profile() is also only compiled on
 * the central (issue #3308).  We therefore implement indicator LED handling
 * ourselves, reactively from events only.
 *
 * The transport for indicator state is already built into ZMK:
 *   - Central: listens for zmk_hid_indicators_changed, writes the bitmask to
 *     each peripheral over a dedicated GATT characteristic.
 *   - Peripheral: service.c receives the write and raises zmk_hid_indicators_changed
 *     locally.  We listen to that event here and drive the GPIO LEDs from it.
 *   - No custom BLE / GATT code is needed in this file.
 *
 * REQUIRED CONFIG (both halves)
 * -----------------------------
 *   CONFIG_ZMK_HID_INDICATORS=y
 *   CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS=y
 *
 * LED LAYOUT
 * ----------
 * Left half (peripheral):
 *   led_caps_lock, led_num_lock, led_scroll_lock
 *   led_left_battery_full, led_left_low_battery
 *
 * Right half (central):
 *   led_layer1, led_layer2, led_layer3
 *   led_bt_profile, led_bt_disconnect, led_bt_pairing
 *   led_right_battery_full, led_right_low_battery
 *
 * led_bt_profile is OFF when BLE profile 0 is active, ON when profile 1 is active.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <zmk/battery.h>

/*
 * HID indicator support.
 * hid_indicators_types.h and the hid_indicators_changed event are available on
 * both the central and the peripheral when CONFIG_ZMK_HID_INDICATORS=y.
 * On the peripheral, the indicator state arrives via
 * CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS; we must not call
 * zmk_hid_indicators_get_current_profile() here because that function is only
 * compiled on the central (issue #3308).
 */
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/hid_indicators_types.h>
#include <zmk/events/hid_indicators_changed.h>

/*
 * Bitmask positions from the USB HID 1.21 spec, Keyboard page, output report.
 * zmk_hid_indicators_t is a uint8_t.
 */
#define HID_IND_NUM_LOCK    BIT(0)
#define HID_IND_CAPS_LOCK   BIT(1)
#define HID_IND_SCROLL_LOCK BIT(2)
#endif /* CONFIG_ZMK_HID_INDICATORS */

LOG_MODULE_REGISTER(status_leds, CONFIG_ZMK_LOG_LEVEL);

/* =========================================================================
 * GPIO specs
 * ========================================================================= */

#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
/* Left (peripheral) half */
static const struct gpio_dt_spec left_leds[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_caps_lock),         gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_num_lock),          gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_scroll_lock),       gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_left_battery_full), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_left_low_battery),  gpios),
};

enum left_led_index {
    LEFT_LED_CAPS = 0,
    LEFT_LED_NUM,
    LEFT_LED_SCROLL,
    LEFT_LED_BATTERY_FULL,
    LEFT_LED_LOW_BATTERY,
};
#endif /* !CONFIG_ZMK_SPLIT_ROLE_CENTRAL */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
/* Right (central) half */
static const struct gpio_dt_spec right_leds[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_layer1),             gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_layer2),             gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_layer3),             gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_bt_profile),         gpios),  /* OFF=profile 0, ON=profile 1 */
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_bt_disconnect),      gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_bt_pairing),         gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_right_battery_full), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_right_low_battery),  gpios),
};

enum right_led_index {
    RIGHT_LED_LAYER1 = 0,
    RIGHT_LED_LAYER2,
    RIGHT_LED_LAYER3,
    RIGHT_LED_BT_PROFILE,   /* lit when BLE profile 1 is active */
    RIGHT_LED_BT_DISCONN,
    RIGHT_LED_PAIRING,
    RIGHT_LED_BATTERY_FULL,
    RIGHT_LED_LOW_BATTERY,
};
#endif /* CONFIG_ZMK_SPLIT_ROLE_CENTRAL */

/* =========================================================================
 * Helpers
 * ========================================================================= */

static void set_led(const struct gpio_dt_spec *led, bool state) {
    if (!device_is_ready(led->port)) {
        return;
    }
    gpio_pin_set_dt(led, state ? 1 : 0);
}

/* =========================================================================
 * Central-only: layer LEDs
 * ========================================================================= */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

#define HIGHEST_LAYER (ZMK_KEYMAP_LAYERS_LEN - 1)

static uint8_t get_highest_layer(zmk_keymap_layers_state_t state) {
    for (int8_t i = HIGHEST_LAYER; i >= 0; i--) {
        if (state & BIT(i)) {
            return (uint8_t)i;
        }
    }
    return 0;
}

static void update_layer_leds(void) {
    uint8_t layer = get_highest_layer(zmk_keymap_layer_state());
    set_led(&right_leds[RIGHT_LED_LAYER1], layer == 1);
    set_led(&right_leds[RIGHT_LED_LAYER2], layer == 2);
    set_led(&right_leds[RIGHT_LED_LAYER3], layer == 3);
}

#endif /* CONFIG_ZMK_SPLIT_ROLE_CENTRAL */

/* =========================================================================
 * Peripheral-only: HID indicator LEDs
 *
 * zmk_hid_indicators_get_current_profile() is only compiled on the central
 * (issue #3308), so we cannot query indicator state at init time.  Instead we
 * keep our own copy of the last-received bitmask and update the LEDs whenever
 * a zmk_hid_indicators_changed event arrives from the split transport.
 *
 * On boot the LEDs start off, which is correct: the host will send an
 * indicator update as soon as the keyboard reports itself connected.
 * ========================================================================= */

#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

/*
 * Local copy of the last indicator bitmask received from the central.
 * Starts at 0 (all off) and is updated in the event listener below.
 */
static zmk_hid_indicators_t current_indicators = 0;

static void update_hid_indicator_leds(zmk_hid_indicators_t indicators) {
    set_led(&left_leds[LEFT_LED_CAPS],   (indicators & HID_IND_CAPS_LOCK)   != 0);
    set_led(&left_leds[LEFT_LED_NUM],    (indicators & HID_IND_NUM_LOCK)    != 0);
    set_led(&left_leds[LEFT_LED_SCROLL], (indicators & HID_IND_SCROLL_LOCK) != 0);
}

#endif /* !CENTRAL && HID_INDICATORS */

/* =========================================================================
 * Battery LEDs (both halves)
 * ========================================================================= */

#define LOW_BATTERY_THRESHOLD 15
#define BATTERY_FULL          100

static void update_battery_leds(void) {
    uint8_t level = zmk_battery_state_of_charge();

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    set_led(&right_leds[RIGHT_LED_BATTERY_FULL], level == BATTERY_FULL);
    set_led(&right_leds[RIGHT_LED_LOW_BATTERY],  level <= LOW_BATTERY_THRESHOLD);
#else
    set_led(&left_leds[LEFT_LED_BATTERY_FULL], level == BATTERY_FULL);
    set_led(&left_leds[LEFT_LED_LOW_BATTERY],  level <= LOW_BATTERY_THRESHOLD);
#endif
}

/* =========================================================================
 * Central-only: BLE profile and connection status LEDs
 * ========================================================================= */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

/*
 * Drive led_bt_profile from the active profile index.
 *
 * The index is read directly from the zmk_ble_active_profile_changed event
 * payload (ev->index) in the event listener, so we never need to call a
 * separate zmk_ble_active_profile_index() getter.
 *
 * Semantics: OFF = profile 0, ON = profile 1.
 */
static void update_bt_profile_led(uint8_t profile_index) {
    set_led(&right_leds[RIGHT_LED_BT_PROFILE], profile_index == 1);
}

static void update_main_connection_status(void) {
    bool bt_connected = zmk_ble_active_profile_is_connected();
    set_led(&right_leds[RIGHT_LED_BT_DISCONN], !bt_connected);

    /* Pairing mode: profile slot is open (unbonded) but not yet connected */
    bool pairing = zmk_ble_active_profile_is_open() && !bt_connected;
    set_led(&right_leds[RIGHT_LED_PAIRING], pairing);
}

#endif /* CONFIG_ZMK_SPLIT_ROLE_CENTRAL */

/* =========================================================================
 * Event listeners
 * ========================================================================= */

/* --- Layer (central only) --- */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static int status_led_layer_event_listener(const zmk_event_t *eh) {
    if (as_zmk_layer_state_changed(eh) == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    update_layer_leds();
    return ZMK_EV_EVENT_BUBBLE;
}
#endif

/* --- Battery (both halves) --- */
static int status_led_battery_event_listener(const zmk_event_t *eh) {
    if (as_zmk_battery_state_changed(eh) == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    update_battery_leds();
    return ZMK_EV_EVENT_BUBBLE;
}

/* --- BLE host profile change (central only) ---
 *
 * Handles both profile switching (ev->index tells us which profile is now
 * active) and connection-state changes that ZMK re-fires as this event.
 * Reading ev->index here is the correct, public way to obtain the active
 * profile index without calling any internal ble.c helpers.
 */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static int status_led_ble_event_listener(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev =
        as_zmk_ble_active_profile_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    update_bt_profile_led(ev->index);
    update_main_connection_status();
    return ZMK_EV_EVENT_BUBBLE;
}
#endif

/*
 * --- HID indicators (peripheral only) ---
 *
 * This event is raised on the peripheral by service.c after it receives a
 * GATT write from the central (see CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS).
 *
 * We must not call zmk_hid_indicators_get_current_profile() here; it is not
 * compiled on the peripheral (issue #3308).  The event payload carries the
 * full bitmask, so we use that directly and keep our own local copy.
 */
#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
static int status_led_hid_indicators_event_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    current_indicators = ev->indicators;
    update_hid_indicator_leds(current_indicators);
    return ZMK_EV_EVENT_BUBBLE;
}
#endif

/* =========================================================================
 * Subscription registration
 * ========================================================================= */

ZMK_LISTENER(status_leds_battery, status_led_battery_event_listener);
ZMK_SUBSCRIPTION(status_leds_battery, zmk_battery_state_changed);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
ZMK_LISTENER(status_leds_layer, status_led_layer_event_listener);
ZMK_SUBSCRIPTION(status_leds_layer, zmk_layer_state_changed);

ZMK_LISTENER(status_leds_ble, status_led_ble_event_listener);
ZMK_SUBSCRIPTION(status_leds_ble, zmk_ble_active_profile_changed);
#endif

#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
ZMK_LISTENER(status_leds_hid_indicators, status_led_hid_indicators_event_listener);
ZMK_SUBSCRIPTION(status_leds_hid_indicators, zmk_hid_indicators_changed);
#endif

/* =========================================================================
 * Initialisation
 * ========================================================================= */

static int status_leds_init(void) {
    int ret = 0;

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
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

    /* Apply initial states */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    update_layer_leds();
    update_main_connection_status();

    /*
     * Assume profile 0 at init (LED off).  ZMK fires zmk_ble_active_profile_changed
     * during startup after loading the persisted active_profile from flash, so
     * the LED will self-correct within milliseconds of boot without any
     * explicit getter call here.
     */
    update_bt_profile_led(0);
#endif

    /*
     * Indicator LEDs on the peripheral start off (current_indicators == 0).
     * We cannot query the current state from hid_indicators.c because
     * zmk_hid_indicators_get_current_profile() is not compiled on the peripheral.
     * The host will push an indicator update when it sees the keyboard connect,
     * which will fire the zmk_hid_indicators_changed event and correct any
     * mismatch within a short time.
     */
#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
    update_hid_indicator_leds(current_indicators);
#endif

    update_battery_leds();

    LOG_INF("Status LEDs initialised");
    return 0;
}

SYS_INIT(status_leds_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
/*
 * status_leds.c — Custom LED status indicators for the Aluminum split keyboard.
 *
 * HID INDICATOR APPROACH
 * ----------------------
 * ZMK's built-in indicator_leds.c cannot compile on a split peripheral because
 * it calls zmk_endpoint_is_connected(), which is not compiled there (issue #3307).
 * Additionally, zmk_hid_indicators_get_current_profile() is also only compiled on
 * the central (issue #3308).  We therefore implement indicator LED handling
 * ourselves, reactively from events only.
 *
 * The transport for indicator state is already built into ZMK:
 *   - Central: listens for zmk_hid_indicators_changed, writes the bitmask to
 *     each peripheral over a dedicated GATT characteristic.
 *   - Peripheral: service.c receives the write and raises zmk_hid_indicators_changed
 *     locally.  We listen to that event here and drive the GPIO LEDs from it.
 *   - No custom BLE / GATT code is needed in this file.
 *
 * REQUIRED CONFIG (both halves)
 * -----------------------------
 *   CONFIG_ZMK_HID_INDICATORS=y
 *   CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS=y
 *
 * LED LAYOUT
 * ----------
 * Left half (peripheral):
 *   led_caps_lock, led_num_lock, led_scroll_lock
 *   led_left_battery_full, led_left_low_battery
 *
 * Right half (central):
 *   led_layer1, led_layer2, led_layer3
 *   led_split_disconnect, led_bt_disconnect, led_bt_pairing
 *   led_right_battery_full, led_right_low_battery
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/split_peripheral_status_changed.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <zmk/battery.h>

/*
 * HID indicator support.
 * hid_indicators_types.h and the hid_indicators_changed event are available on
 * both the central and the peripheral when CONFIG_ZMK_HID_INDICATORS=y.
 * On the peripheral, the indicator state arrives via
 * CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS; we must not call
 * zmk_hid_indicators_get_current_profile() here because that function is only
 * compiled on the central (issue #3308).
 */
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/hid_indicators_types.h>
#include <zmk/events/hid_indicators_changed.h>

/*
 * Bitmask positions from the USB HID 1.21 spec, Keyboard page, output report.
 * zmk_hid_indicators_t is a uint8_t.
 */
#define HID_IND_NUM_LOCK    BIT(0)
#define HID_IND_CAPS_LOCK   BIT(1)
#define HID_IND_SCROLL_LOCK BIT(2)
#endif /* CONFIG_ZMK_HID_INDICATORS */

LOG_MODULE_REGISTER(status_leds, CONFIG_ZMK_LOG_LEVEL);

/* =========================================================================
 * GPIO specs
 * ========================================================================= */

#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
/* Left (peripheral) half */
static const struct gpio_dt_spec left_leds[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_caps_lock),         gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_num_lock),          gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_scroll_lock),       gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_left_battery_full), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_left_low_battery),  gpios),
};

enum left_led_index {
    LEFT_LED_CAPS = 0,
    LEFT_LED_NUM,
    LEFT_LED_SCROLL,
    LEFT_LED_BATTERY_FULL,
    LEFT_LED_LOW_BATTERY,
};
#endif /* !CONFIG_ZMK_SPLIT_ROLE_CENTRAL */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
/* Right (central) half */
static const struct gpio_dt_spec right_leds[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_layer1),             gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_layer2),             gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_layer3),             gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_split_disconnect),   gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_bt_disconnect),      gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_bt_pairing),         gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_right_battery_full), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_right_low_battery),  gpios),
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
#endif /* CONFIG_ZMK_SPLIT_ROLE_CENTRAL */

/* =========================================================================
 * Helpers
 * ========================================================================= */

static void set_led(const struct gpio_dt_spec *led, bool state) {
    if (!device_is_ready(led->port)) {
        return;
    }
    gpio_pin_set_dt(led, state ? 1 : 0);
}

/* =========================================================================
 * Central-only: layer LEDs
 * ========================================================================= */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

#define HIGHEST_LAYER (ZMK_KEYMAP_LAYERS_LEN - 1)

static uint8_t get_highest_layer(zmk_keymap_layers_state_t state) {
    for (int8_t i = HIGHEST_LAYER; i >= 0; i--) {
        if (state & BIT(i)) {
            return (uint8_t)i;
        }
    }
    return 0;
}

static void update_layer_leds(void) {
    uint8_t layer = get_highest_layer(zmk_keymap_layer_state());
    set_led(&right_leds[RIGHT_LED_LAYER1], layer == 1);
    set_led(&right_leds[RIGHT_LED_LAYER2], layer == 2);
    set_led(&right_leds[RIGHT_LED_LAYER3], layer == 3);
}

#endif /* CONFIG_ZMK_SPLIT_ROLE_CENTRAL */

/* =========================================================================
 * Peripheral-only: HID indicator LEDs
 *
 * zmk_hid_indicators_get_current_profile() is only compiled on the central
 * (issue #3308), so we cannot query indicator state at init time.  Instead we
 * keep our own copy of the last-received bitmask and update the LEDs whenever
 * a zmk_hid_indicators_changed event arrives from the split transport.
 *
 * On boot the LEDs start off, which is correct: the host will send an
 * indicator update as soon as the keyboard reports itself connected.
 * ========================================================================= */

#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

/*
 * Local copy of the last indicator bitmask received from the central.
 * Starts at 0 (all off) and is updated in the event listener below.
 */
static zmk_hid_indicators_t current_indicators = 0;

static void update_hid_indicator_leds(zmk_hid_indicators_t indicators) {
    set_led(&left_leds[LEFT_LED_CAPS],   (indicators & HID_IND_CAPS_LOCK)   != 0);
    set_led(&left_leds[LEFT_LED_NUM],    (indicators & HID_IND_NUM_LOCK)    != 0);
    set_led(&left_leds[LEFT_LED_SCROLL], (indicators & HID_IND_SCROLL_LOCK) != 0);
}

#endif /* !CENTRAL && HID_INDICATORS */

/* =========================================================================
 * Battery LEDs (both halves)
 * ========================================================================= */

#define LOW_BATTERY_THRESHOLD 15
#define BATTERY_FULL          100

static void update_battery_leds(void) {
    uint8_t level = zmk_battery_state_of_charge();

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    set_led(&right_leds[RIGHT_LED_BATTERY_FULL], level == BATTERY_FULL);
    set_led(&right_leds[RIGHT_LED_LOW_BATTERY],  level <= LOW_BATTERY_THRESHOLD);
#else
    set_led(&left_leds[LEFT_LED_BATTERY_FULL], level == BATTERY_FULL);
    set_led(&left_leds[LEFT_LED_LOW_BATTERY],  level <= LOW_BATTERY_THRESHOLD);
#endif
}

/* =========================================================================
 * Central-only: BLE / split connection status LEDs
 * ========================================================================= */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

static void update_main_connection_status(void) {
    bool bt_connected = zmk_ble_active_profile_is_connected();
    set_led(&right_leds[RIGHT_LED_BT_DISCONN], !bt_connected);

    /* Pairing mode: profile slot is open (unbonded) but not yet connected */
    bool pairing = zmk_ble_active_profile_is_open() && !bt_connected;
    set_led(&right_leds[RIGHT_LED_PAIRING], pairing);
}

static void update_peripheral_connection_status(bool connected) {
    set_led(&right_leds[RIGHT_LED_SPLIT_DISCONN], !connected);
}

#endif /* CONFIG_ZMK_SPLIT_ROLE_CENTRAL */

/* =========================================================================
 * Event listeners
 * ========================================================================= */

/* --- Layer (central only) --- */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static int status_led_layer_event_listener(const zmk_event_t *eh) {
    if (as_zmk_layer_state_changed(eh) == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    update_layer_leds();
    return ZMK_EV_EVENT_BUBBLE;
}
#endif

/* --- Battery (both halves) --- */
static int status_led_battery_event_listener(const zmk_event_t *eh) {
    if (as_zmk_battery_state_changed(eh) == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    update_battery_leds();
    return ZMK_EV_EVENT_BUBBLE;
}

/* --- Split peripheral connect/disconnect (central only) --- */
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

/* --- BLE host profile change (central only) --- */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static int status_led_ble_event_listener(const zmk_event_t *eh) {
    if (as_zmk_ble_active_profile_changed(eh) == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    update_main_connection_status();
    return ZMK_EV_EVENT_BUBBLE;
}
#endif

/*
 * --- HID indicators (peripheral only) ---
 *
 * This event is raised on the peripheral by service.c after it receives a
 * GATT write from the central (see CONFIG_ZMK_SPLIT_PERIPHERAL_HID_INDICATORS).
 *
 * We must not call zmk_hid_indicators_get_current_profile() here; it is not
 * compiled on the peripheral (issue #3308).  The event payload carries the
 * full bitmask, so we use that directly and keep our own local copy.
 */
#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
static int status_led_hid_indicators_event_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    current_indicators = ev->indicators;
    update_hid_indicator_leds(current_indicators);
    return ZMK_EV_EVENT_BUBBLE;
}
#endif

/* =========================================================================
 * Subscription registration
 * ========================================================================= */

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

#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
ZMK_LISTENER(status_leds_hid_indicators, status_led_hid_indicators_event_listener);
ZMK_SUBSCRIPTION(status_leds_hid_indicators, zmk_hid_indicators_changed);
#endif

/* =========================================================================
 * Initialisation
 * ========================================================================= */

static int status_leds_init(void) {
    int ret = 0;

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
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

    /* Apply initial states */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    update_layer_leds();
    update_main_connection_status();
    update_peripheral_connection_status(false);
#endif

    /*
     * Indicator LEDs on the peripheral start off (current_indicators == 0).
     * We cannot query the current state from hid_indicators.c because
     * zmk_hid_indicators_get_current_profile() is not compiled on the peripheral.
     * The host will push an indicator update when it sees the keyboard connect,
     * which will fire the zmk_hid_indicators_changed event and correct any
     * mismatch within a short time.
     */
#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) && IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
    update_hid_indicator_leds(current_indicators);
#endif

    update_battery_leds();

    LOG_INF("Status LEDs initialised");
    return 0;
}

SYS_INIT(status_leds_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
