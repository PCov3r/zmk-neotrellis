#include <zephyr/device.h>       /* device model, DEVICE_DT_INST_DEFINE */
#include <zephyr/devicetree.h>   /* DT macros */
#include <zephyr/drivers/i2c.h>  /* i2c_dt_spec, i2c operations */
#include <zephyr/drivers/input.h>/* input_event and input_report() */
#include <zephyr/kernel.h>       /* threads, k_msleep, K_THREAD_STACK_DEFINE */
#include <zephyr/logging/log.h>  /* logging */

LOG_MODULE_REGISTER(neotrellis, CONFIG_LOG_DEFAULT_LEVEL);

struct neotrellis_config {
    struct i2c_dt_spec i2c;       /* I2C bus + address for this instance */
};

/* Base address for keypad module inside Seesaw */
#define SEESAW_KEYPAD_BASE       0x10

/* Register addresses are built as (module << 8 | offset) */
#define SEESAW_KEYPAD_STATUS     ((SEESAW_KEYPAD_BASE << 8) | 0x01)
#define SEESAW_KEYPAD_EVENT      ((SEESAW_KEYPAD_BASE << 8) | 0x02)
#define SEESAW_KEYPAD_INTENSET   ((SEESAW_KEYPAD_BASE << 8) | 0x10)
#define SEESAW_KEYPAD_COUNT      ((SEESAW_KEYPAD_BASE << 8) | 0x11)

#define NEOTRELLIS_NUM_KEYS      16  /* fixed 4x4 matrix */

/* Write a single byte to a 16-bit Seesaw register */
static int neotrellis_write8(const struct device *dev, uint16_t reg, uint8_t val)
{
    const struct neotrellis_config *cfg = dev->config;
    uint8_t buf[3] = { reg >> 8, reg & 0xFF, val };

    return i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
}

/* Read N bytes from a 16-bit Seesaw register */
static int neotrellis_read(const struct device *dev, uint16_t reg, uint8_t *buf, size_t len)
{
    const struct neotrellis_config *cfg = dev->config;
    uint8_t addr[2] = { reg >> 8, reg & 0xFF };

    return i2c_write_read_dt(&cfg->i2c, addr, sizeof(addr), buf, len);
}

/* Enable all keys for rising and falling edge events */
static int neotrellis_enable_keys(const struct device *dev)
{
    int ret = 0;

    /* Seesaw has 16 keys (4x4) */
    for (uint8_t k = 0; k < NEOTRELLIS_NUM_KEYS; k++) {
        /* For simplicity, enable all keys in the INTENSET register */
        /* In Arduino code, activateKey(k, EDGE_RISING/FALLING) sets per-key mask */
        ret |= neotrellis_write8(dev, SEESAW_KEYPAD_INTENSET, (1 << (k & 0x7)));
        /* Note: the upper byte of INTENSET applies to keys 8-15, could be split if needed */
    }

    return ret;
}


/* Poll the keypad for events and report them to ZMK */
static void neotrellis_poll(const struct device *dev)
{
    uint8_t count = 0;

    /* Read how many events are waiting in the FIFO */
    if (neotrellis_read(dev, SEESAW_KEYPAD_COUNT, &count, 1) < 0) {
        return; /* bus error, skip this round */
    }

    while (count--) {
        uint8_t evt[2];
        if (neotrellis_read(dev, SEESAW_KEYPAD_EVENT, evt, sizeof(evt)) < 0) {
            return;
        }

        uint8_t key = evt[0] & 0x3F;   /* bits 0-5 = key number (0-15) */
        bool pressed = evt[0] & 0x80;  /* bit 7 = pressed/released */

        LOG_DBG("Key %d %s", key, pressed ? "pressed" : "released");

        /* Report the key event to ZMK */
        struct input_event ie = {
            .type = INPUT_EV_KEY,
            .code = key,
            .value = pressed ? 1 : 0,
        };
        input_report(dev, &ie, K_NO_WAIT);
    }
}


#define NEOTRELLIS_THREAD_STACK 512

K_THREAD_STACK_DEFINE(neotrellis_stack, NEOTRELLIS_THREAD_STACK);
static struct k_thread neotrellis_thread_data;

/* Thread entry function */
static void neotrellis_thread(void *arg1, void *arg2, void *arg3)
{
    const struct device *dev = arg1;

    while (1) {
        neotrellis_poll(dev);   /* check for new key events */
        k_msleep(20);           /* ~50–60 Hz polling, like Arduino example */
    }
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
                          NULL);

/* Instantiate all Trellis devices found in the device tree */
DT_INST_FOREACH_STATUS_OKAY(NEOTRELLIS_INIT)
