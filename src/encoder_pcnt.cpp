#include"encoder_pcnt.hpp"
#include<driver/pcnt.h>
#include<Arduino.h>

#define RIGHT_ENC_PIN_A 32
#define RIGHT_ENC_PIN_B 33
#define LEFT_ENC_PIN_A  22
#define LEFT_ENC_PIN_B  23

const int16_t PCNT_MAX = 32767;
const int16_t PCNT_MIN = -32768;

static void single_encoder_init(pcnt_unit_t unit, int PIN_A, int PIN_B)
{
    pcnt_config_t pcnt_cfg = {};
    pcnt_cfg.pulse_gpio_num = PIN_A;
    pcnt_cfg.ctrl_gpio_num = PIN_B;
    pcnt_cfg.unit = unit;
    pcnt_cfg.channel = PCNT_CHANNEL_0;
    pcnt_cfg.pos_mode = PCNT_COUNT_INC;
    pcnt_cfg.neg_mode = PCNT_COUNT_DIS;
    pcnt_cfg.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_cfg.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_cfg.counter_h_lim = PCNT_MAX;
    pcnt_cfg.counter_l_lim = PCNT_MIN;

    pcnt_unit_config(&pcnt_cfg);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void encoder_init()
{
    pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
    pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);

    single_encoder_init(PCNT_UNIT_0, LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);
    single_encoder_init(PCNT_UNIT_1, RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B);
}

int16_t get_left_encoder_count()
{
    int16_t v = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &v);
    return v;
}

int16_t get_right_encoder_count()
{
    int16_t v = 0;
    pcnt_get_counter_value(PCNT_UNIT_1, &v);
    return v;
}

void clear_right_counter()
{
    pcnt_counter_clear(PCNT_UNIT_0);
}

void clear_left_counter()
{
    pcnt_counter_clear(PCNT_UNIT_1);
}
