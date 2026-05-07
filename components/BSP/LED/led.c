#include "led.h"

void bled_init(void)
{
    gpio_config_t io_conf = {0};
    io_conf.pin_bit_mask = (1ULL << BLED_GPIO_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    BLED(1);
}

static bool s_bled_ready = false;

static inline void ensure_bled_ready(void)
{
    if (!s_bled_ready) {
        bled_init();
        s_bled_ready = true;
    }
}

void bled_set(bool on)
{
    ensure_bled_ready();
    /* 电路为高电平点亮（基极上拉导通） */
    BLED(on ? 1 : 0);
}

void bled_on(void)
{
    bled_set(true);
}

void bled_off(void)
{
    bled_set(false);
}

