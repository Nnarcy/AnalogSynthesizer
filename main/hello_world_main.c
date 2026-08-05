/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/i2s_std.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"

#define SAMPLE_RATE_HZ      44100
#define TONE_FREQUENCY_HZ   440
#define MAX_TONE_AMPLITUDE  4000 // MAXIMUM VOLUME
#define FRAMES_PER_BUFFER   256
#define ADC_MAX_READING     4095

#define I2S_WSEL_GPIO       GPIO_NUM_4
#define I2S_DATA_GPIO       GPIO_NUM_5
#define I2S_BCLK_GPIO       GPIO_NUM_6
#define POTENTIOMETER_GPIO  GPIO_NUM_0
#define POTENTIOMETER_ADC_CHANNEL ADC_CHANNEL_0

static i2s_chan_handle_t tx_channel;
static adc_oneshot_unit_handle_t adc_handle;
static int16_t audio_buffer[FRAMES_PER_BUFFER * 2];

static void initialize_i2s(void)
{
    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&channel_config, &tx_channel, NULL));

    i2s_std_config_t standard_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_GPIO,
            .ws = I2S_WSEL_GPIO,
            .dout = I2S_DATA_GPIO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_channel, &standard_config));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_channel));
}

static void initialize_volume_control(void)
{
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc_handle, POTENTIOMETER_ADC_CHANNEL, &channel_config));
}

static int read_tone_amplitude(void)
{
    static int filtered_reading;
    int raw_reading;

    ESP_ERROR_CHECK(adc_oneshot_read(
        adc_handle, POTENTIOMETER_ADC_CHANNEL, &raw_reading));

    // Smooth small ADC fluctuations so the volume does not sound rough.
    filtered_reading = (filtered_reading * 7 + raw_reading) / 8;

    // Treat readings near ground as complete silence.
    if (filtered_reading < 20) {
        return 0;
    }

    return filtered_reading * MAX_TONE_AMPLITUDE / ADC_MAX_READING;
}

static void fill_sine_buffer(float *phase, int amplitude)
{
    const float phase_step =
        2.0f * (float)M_PI * TONE_FREQUENCY_HZ / SAMPLE_RATE_HZ;

    for (int frame = 0; frame < FRAMES_PER_BUFFER; frame++) {
        int16_t sample = (int16_t)(sinf(*phase) * amplitude);

        // I2S stereo samples are interleaved: left, right, left, right...
        audio_buffer[frame * 2] = sample;
        audio_buffer[frame * 2 + 1] = sample;

        *phase += phase_step;
        if (*phase >= 2.0f * (float)M_PI) {
            *phase -= 2.0f * (float)M_PI;
        }
    }
}

void app_main(void)
{
    initialize_i2s();
    initialize_volume_control();
    printf("Playing a %d Hz tone on GPIOs WSEL=%d, DATA=%d, BCLK=%d; "
           "volume potentiometer on GPIO %d\n",
           TONE_FREQUENCY_HZ,
           I2S_WSEL_GPIO,
           I2S_DATA_GPIO,
           I2S_BCLK_GPIO,
           POTENTIOMETER_GPIO);

    float phase = 0.0f;

    while (true) {
        size_t bytes_written;
        int amplitude = read_tone_amplitude();

        fill_sine_buffer(&phase, amplitude);
        ESP_ERROR_CHECK(i2s_channel_write(tx_channel,
                                          audio_buffer,
                                          sizeof(audio_buffer),
                                          &bytes_written,
                                          portMAX_DELAY));
    }
}
