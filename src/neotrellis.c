#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/input.h>
#include <zmk/hid.h>

LOG_MODULE_REGISTER(neotrellis, CONFIG_LOG_DEFAULT_LEVEL);

#define NEOTRELLIS_NUM_KEYS 16
#define SEESAW_KEYPAD_BASE       0x10
#define SEESAW_KEYPAD_STATUS     ((SEESAW_KEYPAD_BASE << 8) | 0x01)
#define SEESAW_KEYPAD_EVENT      ((SEESAW_KEYPAD_BASE << 8) | 0x02)
#define SEESAW_KEYPAD_INTENSET   ((SEESAW_KEYPAD_BASE << 8) | 0x10)
#define SEESAW_KEYPAD_COUNT      ((SEESAW_KEYPAD_BASE << 8) | 0x11)

struct neotrellis_config {
    struct i2c_dt_spec i2c;
};

struct neotrellis_data {
    struct k_thread thread_data;
    K_THREAD_STACK_MEMBER(stack, 512);
};

static int neotrellis_write8(const struct device *dev, uint16_t reg, uint8_t val) {
    const struct neotrellis_config *cfg = dev->config;
    uint8_t buf[3] = { reg >> 8, reg & 0xFF, val };
    return i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
}

static int neotrellis_read(const struct device *dev, uint16_t reg, uint8_t *buf, size_t len) {
    const struct neotrellis_config *cfg = dev->config;
    uint8_t addr[2] = { reg >> 8, reg & 0xFF };
    return i2c_write_read_dt(&cfg->i2c, addr, sizeof(addr), buf, len);
}

static int neotrellis_enable_keys(const struct device *dev) {
    for (uint8_t k = 0; k < NEOTRELLIS_NUM_KEYS; k++) {
        neotrellis_write8(dev, SEESAW_KEYPAD_INTENSET, (1 << (k & 0x7)));
    }
    return 0;
}

static void neotrellis_poll(const struct device *dev) {
    uint8_t count = 0;
    if (neotrellis_read(dev, SEESAW_KEYPAD_COUNT, &count, 1) < 0) return;

    while (count--) {
        uint8_t evt[2];
        if (neotrellis_read(dev, SEESAW_KEYPAD_EVENT, evt, sizeof(evt)) < 0) return;

        uint8_t key = evt[0] & 0x3F;
        bool pressed = evt[0] & 0x80;

        LOG_DBG("Key %d %s", key, pressed ? "pressed" : "released");

        uint16_t keycode = key; // map key 0-15 to ZMK keymap directly
        if (pressed) {
            zmk_hid_keypad_press(keycode);
        } else {
            zmk_hid_keypad_release(keycode);
        }
    }
}

static void neotrellis_thread(void *arg1, void *arg2, void *arg3) {
    const struct device *dev = arg1;
    while (1) {
        neotrellis_poll(dev);
        k_msleep(20);
    }
}

static int neotrellis_init(const struct device *dev) {
    neotrellis_enable_keys(dev);

    struct neotrellis_data *data = dev->data;
    k_thread_create(&data->thread_data, data->stack, K_THREAD_STACK_SIZEOF(data->stack),
                    neotrellis_thread, (void *)dev, NULL, NULL,
                    K_PRIO_COOP(10), 0, K_NO_WAIT);

    return 0;
}

#define NEOTRELLIS_INIT(inst) \
    static struct neotrellis_data neotrellis_data_##inst; \
    static const struct neotrellis_config neotrellis_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, neotrellis_init, NULL, \
                          &neotrellis_data_##inst, \
                          &neotrellis_config_##inst, \
                          POST_KERNEL, \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
                          NULL); \
    ZMK_INPUT_DEVICES_DEFINE(trellis##inst, neotrellis_init);

DT_INST_FOREACH_STATUS_OKAY(NEOTRELLIS_INIT)
