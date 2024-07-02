// Copyright 2020 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Fsk9600Demodulator.hpp"
#include "Goertzel.h"
#include "AudioInput.hpp"
#include "GPIO.hpp"
#include "Log.h"
#include "power.h"

namespace mobilinkd { namespace tnc {

hdlc::IoFrame* Fsk9600Demodulator::operator()(const q15_t* samples)
{
    hdlc::IoFrame* result = nullptr;

    auto filtered = demod_filter.filter(const_cast<q15_t* >(samples));

    for (size_t i = 0; i != ADC_BLOCK_SIZE; ++i)
    {
        auto sample = filtered[i];

        bool bit = sample >= 0;
        auto pll = pll_(bit);

        if (pll.sample)
        {
            locked_ = pll.locked;

            // We will only ever get one frame because there are
            // not enough bits in a block for more than one.
            if (result) {
                auto tmp = hdlc_decoder_(nrzi_.decode(lfsr_(bit)), locked_);
                if (tmp) hdlc::release(tmp);
            } else {
                result = hdlc_decoder_(nrzi_.decode(lfsr_(bit)), locked_);
#ifdef KISS_LOGGING
                if (result) {
                    INFO("samples = %ld, mean = %d, dev = %d",
                        snr_.samples, int(snr_.mean), int(snr_.stdev()));
                    INFO("SNR = %dmB", int(snr_.SNR() * 100.0f));
                    snr_.reset();
                }
#endif
            }

#ifdef KISS_LOGGING
            if (hdlc_decoder_.active())
            {
                if (!decoding_)
                {
                    snr_.reset();
                    decoding_ = true;
                }
                snr_.capture(float(abs(sample)));
            } else {
                decoding_ = false;
            }
#endif
        }

    }
    return result;
}

/*
 * Return twist as a the difference in dB between mark and space.  The
 * expected values are about 0dB for discriminator output and about 5.5dB
 * for de-emphasized audio.
 */
float Fsk9600Demodulator::readTwist()
{
    TNC_DEBUG("enter Fsk9600Demodulator::readTwist");

    float g120 = 0.0f;
    float g4800 = 0.0f;

    GoertzelFilter<ADC_BLOCK_SIZE, SAMPLE_RATE> gf120(120.0, 0);
    GoertzelFilter<ADC_BLOCK_SIZE, SAMPLE_RATE> gf4800(4800.0, 0);

    const uint32_t AVG_SAMPLES = 160;

    startADC(416, ADC_BLOCK_SIZE);

    for (uint32_t i = 0; i != AVG_SAMPLES; ++i)
    {
        uint32_t count = 0;
        while (count < ADC_BLOCK_SIZE)
        {
            osEvent evt = osMessageGet(adcInputQueueHandle, osWaitForever);
            if (evt.status != osEventMessage)
                continue;

            auto block = (audio::adc_pool_type::chunk_type*) evt.value.p;
            uint16_t* data = (uint16_t*) block->buffer;
            gf120(data, ADC_BLOCK_SIZE);
            gf4800(data, ADC_BLOCK_SIZE);

            audio::adcPool.deallocate(block);

            count += ADC_BLOCK_SIZE;
        }

        g120 += (gf120 / count);
        g4800 += (gf4800 / count);

        gf120.reset();
        gf4800.reset();
    }

    IDemodulator::stopADC();

    g120 = 10.0f * log10f(g120 / AVG_SAMPLES);
    g4800 = 10.0f * log10f(g4800 / AVG_SAMPLES);

    auto result = g120 - g4800;

    INFO("9600 Twist = %d / 100 (%d - %d)", int(result * 100), int(g120 * 100),
        int(g4800 * 100));

    TNC_DEBUG("exit Fsk9600Demodulator::readTwist");
    return result;
}

uint32_t Fsk9600Demodulator::readBatteryLevel()
{
#if defined(STM32L4P5xx) || defined(STM32L4Q5xx)
    return get_bat_level();
#elif !(defined(NUCLEOTNC))
    TNC_DEBUG("enter Fsk9600Demodulator::readBatteryLevel");

    ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&BATTERY_ADC_HANDLE, &sConfig) != HAL_OK)
        CxxErrorHandler();

    htim6.Init.Period = 48000;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) CxxErrorHandler();

    if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
        CxxErrorHandler();

    if (HAL_ADC_Start(&BATTERY_ADC_HANDLE) != HAL_OK) CxxErrorHandler();
    if (HAL_ADC_PollForConversion(&BATTERY_ADC_HANDLE, 3) != HAL_OK) CxxErrorHandler();
    auto vrefint = HAL_ADC_GetValue(&BATTERY_ADC_HANDLE);
    if (HAL_ADC_Stop(&BATTERY_ADC_HANDLE) != HAL_OK) CxxErrorHandler();

    // Disable battery charging while measuring battery voltage.
    auto usb_ce = gpio::USB_CE::get();
    gpio::USB_CE::on();

    gpio::BAT_DIVIDER::off();
    DELAY(1);

    sConfig.Channel = BATTERY_ADC_CHANNEL;
    if (HAL_ADC_ConfigChannel(&BATTERY_ADC_HANDLE, &sConfig) != HAL_OK)
        CxxErrorHandler();

    uint32_t vbat = 0;
    if (HAL_ADC_Start(&BATTERY_ADC_HANDLE) != HAL_OK) CxxErrorHandler();
    for (size_t i = 0; i != 8; ++i)
    {
        if (HAL_ADC_PollForConversion(&BATTERY_ADC_HANDLE, 1) != HAL_OK) CxxErrorHandler();
        vbat += HAL_ADC_GetValue(&BATTERY_ADC_HANDLE);
    }

    vbat /= 8;

    if (HAL_ADC_Stop(&BATTERY_ADC_HANDLE) != HAL_OK) CxxErrorHandler();
    if (HAL_TIM_Base_Stop(&htim6) != HAL_OK)
        CxxErrorHandler();

    gpio::BAT_DIVIDER::on();

    // Restore battery charging state.
    if (!usb_ce) gpio::USB_CE::off();

    INFO("Vref = %lu", vrefint);
    INFO("Vbat = %lu (raw)", vbat);

    // Order of operations is important to avoid underflow.
    vbat *= 6600;
    vbat /= (VREF + 1);

    uint32_t vref = ((vrefint * 3300) + (VREF / 2)) / VREF;

    INFO("Vref = %lumV", vref)
    INFO("Vbat = %lumV", vbat);

    TNC_DEBUG("exit Fsk9600Demodulator::readBatteryLevel");
    return vbat;
#else
    return 0;
#endif
}

const Fsk9600Demodulator::bpf_bank_type Fsk9600Demodulator::bpf_bank = {{
    // -3dB, gain = 0.251188643150958, actual = -1.67dB
    {{
            0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
            1,     1,     2,     2,     3,     4,     4,     5,     6,     7,     8,     9,
           10,    12,    14,    16,    19,    23,    28,    34,    41,    50,    59,    69,
           79,    91,   102,   113,   123,   133,   141,   147,   151,   153,   153,   151,
          147,   141,   133,   123,   113,   102,    91,    79,    69,    59,    50,    41,
           34,    28,    23,    19,    16,    14,    12,    10,     9,     8,     7,     6,
            5,     4,     4,     3,     2,     2,     1,     1,     0,     0,     0,     0,
            0,     0,     0,     0,     0,     0,     0,     0,
    }},
    // -2dB, gain = 0.3981071705534972, actual = -1.22dB
    {{
            0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
            1,     1,     2,     2,     3,     3,     4,     4,     4,     4,     4,     4,
            4,     5,     5,     7,     9,    13,    18,    24,    33,    43,    54,    67,
           81,    96,   111,   127,   141,   154,   165,   174,   180,   183,   183,   180,
          174,   165,   154,   141,   127,   111,    96,    81,    67,    54,    43,    33,
           24,    18,    13,     9,     7,     5,     5,     4,     4,     4,     4,     4,
            4,     4,     3,     3,     2,     2,     1,     1,     0,     0,     0,     0,
            0,     0,     0,     0,     0,     0,     0,     0,
    }},
    // -1dB, gain = 0.6309573444801932, actual = -0.69dB
    {{
            0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
            1,     1,     2,     2,     3,     3,     3,     2,     1,     0,     0,    -2,
           -4,    -6,    -7,    -7,    -5,    -3,     2,     9,    19,    32,    47,    64,
           84,   105,   127,   148,   169,   188,   204,   217,   226,   231,   231,   226,
          217,   204,   188,   169,   148,   127,   105,    84,    64,    47,    32,    19,
            9,     2,    -3,    -5,    -7,    -7,    -6,    -4,    -2,     0,     0,     1,
            2,     3,     3,     3,     2,     2,     1,     1,     0,     0,     0,     0,
            0,     0,     0,     0,     0,     0,     0,     0,
    }},
    // 0dB, gain = 1.0, actual = -0.03dB
    {{
            0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1,
            1,     2,     2,     2,     2,     2,     1,     0,    -2,    -6,   -10,   -14,
          -19,   -23,   -27,   -30,   -30,   -29,   -24,   -15,    -2,    14,    35,    60,
           88,   119,   151,   183,   213,   241,   266,   285,   299,   306,   306,   299,
          285,   266,   241,   213,   183,   151,   119,    88,    60,    35,    14,    -2,
          -15,   -24,   -29,   -30,   -30,   -27,   -23,   -19,   -14,   -10,    -6,    -2,
            0,     1,     2,     2,     2,     2,     2,     1,     1,     0,     0,     0,
            0,     0,     0,     0,     0,     0,     0,     0,
    }},
    // 1dB, gain = 1.5848931924611136, actual = 0.70dB
    {{
            0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1,
            2,     2,     2,     3,     2,     1,    -1,    -4,   -10,   -16,   -24,   -33,
          -42,   -51,   -60,   -66,   -70,   -70,   -65,   -54,   -37,   -13,    17,    53,
           95,   141,   189,   237,   284,   327,   364,   393,   414,   424,   424,   414,
          393,   364,   327,   284,   237,   189,   141,    95,    53,    17,   -13,   -37,
          -54,   -65,   -70,   -70,   -66,   -60,   -51,   -42,   -33,   -24,   -16,   -10,
           -4,    -1,     1,     2,     3,     2,     2,     2,     1,     0,     0,     0,
            0,     0,     0,     0,     0,     0,     0,     0,
    }},
    // 2dB, gain = 2.51188643150958, actual = 1.42dB
    {{
            0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1,     1,
            2,     3,     3,     3,     1,     0,    -5,   -12,   -21,   -33,   -47,   -62,
          -79,   -96,  -112,  -125,  -133,  -135,  -130,  -116,   -91,   -57,   -11,    43,
          106,   175,   249,   323,   395,   461,   519,   565,   596,   613,   613,   596,
          565,   519,   461,   395,   323,   249,   175,   106,    43,   -11,   -57,   -91,
         -116,  -130,  -135,  -133,  -125,  -112,   -96,   -79,   -62,   -47,   -33,   -21,
          -12,    -5,     0,     1,     3,     3,     3,     2,     1,     1,     0,     0,
            0,     0,     0,     0,     0,     0,     0,     0,
    }},
    // 3dB, gain = 3.9810717055349722, actual = 2.07dB
    {{
            0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1,     2,
            3,     4,     4,     3,     0,    -4,   -12,   -24,   -39,   -59,   -83,  -109,
         -138,  -167,  -194,  -217,  -233,  -239,  -234,  -214,  -178,  -126,   -58,    26,
          123,   231,   344,   460,   572,   675,   765,   836,   886,   911,   911,   886,
          836,   765,   675,   572,   460,   344,   231,   123,    26,   -58,  -126,  -178,
         -214,  -234,  -239,  -233,  -217,  -194,  -167,  -138,  -109,   -83,   -59,   -39,
          -24,   -12,    -4,     0,     3,     4,     4,     3,     2,     1,     0,     0,
            0,     0,     0,     0,     0,     0,     0,     0,
    }},
    // 4dB, gain = 6.309573444801933, actual = 2.59dB
    {{
            0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     2,     3,
            5,     6,     6,     4,     0,    -9,   -23,   -42,   -68,  -101,  -140,  -184,
         -231,  -279,  -324,  -363,  -391,  -404,  -398,  -369,  -316,  -237,  -131,     0,
          150,   318,   496,   676,   852,  1013,  1154,  1266,  1344,  1385,  1385,  1344,
         1266,  1154,  1013,   852,   676,   496,   318,   150,     0,  -131,  -237,  -316,
         -369,  -398,  -404,  -391,  -363,  -324,  -279,  -231,  -184,  -140,  -101,   -68,
          -42,   -23,    -9,     0,     4,     6,     6,     5,     3,     2,     0,     0,
            0,     0,     0,     0,     0,     0,     0,     0,
    }},
    // 5dB, gain = 10.0, actual = 2.99dB
    {{
            1,     0,     0,     0,     0,     0,     0,     0,     0,     1,     3,     5,
            7,     9,     8,     5,    -3,   -17,   -40,   -72,  -114,  -168,  -231,  -302,
         -378,  -456,  -530,  -594,  -641,  -664,  -658,  -616,  -535,  -411,  -247,   -43,
          193,   456,   736,  1020,  1295,  1550,  1771,  1948,  2071,  2135,  2135,  2071,
         1948,  1771,  1550,  1295,  1020,   736,   456,   193,   -43,  -247,  -411,  -535,
         -616,  -658,  -664,  -641,  -594,  -530,  -456,  -378,  -302,  -231,  -168,  -114,
          -72,   -40,   -17,    -3,     5,     8,     9,     7,     5,     3,     1,     0,
            0,     0,     0,     0,     0,     0,     0,     1,
    }},
    // 6dB, gain = 15.848931924611133, actual = 3.28dB
    {{
            1,     1,     0,     0,     0,    -1,    -1,    -1,     0,     2,     4,     8,
           11,    13,    12,     6,    -7,   -31,   -67,  -119,  -187,  -273,  -374,  -489,
         -612,  -738,  -857,  -960, -1037, -1077, -1070, -1007,  -881,  -689,  -431,  -111,
          261,   676,  1116,  1564,  1999,  2400,  2750,  3029,  3223,  3323,  3323,  3223,
         3029,  2750,  2400,  1999,  1564,  1116,   676,   261,  -111,  -431,  -689,  -881,
        -1007, -1070, -1077, -1037,  -960,  -857,  -738,  -612,  -489,  -374,  -273,  -187,
         -119,   -67,   -31,    -7,     6,    12,    13,    11,     8,     4,     2,     0,
           -1,    -1,    -1,     0,     0,     0,     1,     1,
    }},
    // 7dB, gain = 25.118864315095795, actual = 3.48dB
    {{
            2,     2,     1,     0,    -1,    -2,    -2,    -1,     0,     3,     7,    12,
           17,    20,    18,     8,   -13,   -52,  -110,  -193,  -303,  -440,  -602,  -785,
         -983, -1183, -1375, -1541, -1666, -1732, -1723, -1626, -1429, -1128,  -722,  -218,
          369,  1024,  1719,  2426,  3113,  3748,  4300,  4741,  5049,  5207,  5207,  5049,
         4741,  4300,  3748,  3113,  2426,  1719,  1024,   369,  -218,  -722, -1128, -1429,
        -1626, -1723, -1732, -1666, -1541, -1375, -1183,  -983,  -785,  -602,  -440,  -303,
         -193,  -110,   -52,   -13,     8,    18,    20,    17,    12,     7,     3,     0,
           -1,    -2,    -2,    -1,     0,     1,     2,     2,
    }},
    // 8dB, gain = 39.810717055349734, actual = 3.61dB
    {{
            4,     3,     1,     0,    -1,    -3,    -3,    -3,     0,     4,    11,    20,
           27,    32,    28,    12,   -23,   -85,  -179,  -311,  -486,  -704,  -963, -1255,
        -1570, -1890, -2196, -2462, -2662, -2770, -2759, -2607, -2299, -1824, -1184,  -388,
          541,  1575,  2674,  3792,  4879,  5884,  6758,  7456,  7942,  8192,  8192,  7942,
         7456,  6758,  5884,  4879,  3792,  2674,  1575,   541,  -388, -1184, -1824, -2299,
        -2607, -2759, -2770, -2662, -2462, -2196, -1890, -1570, -1255,  -963,  -704,  -486,
         -311,  -179,   -85,   -23,    12,    28,    32,    27,    20,    11,     4,     0,
           -3,    -3,    -3,    -1,     0,     1,     3,     4,
    }},
    // 9dB, gain = 63.09573444801933, actual = 3.69dB
    {{
            6,     4,     2,     0,    -3,    -5,    -6,    -4,     0,     7,    18,    31,
           43,    49,    44,    17,   -39,  -137,  -287,  -499,  -776, -1123, -1535, -2000,
        -2500, -3010, -3497, -3921, -4241, -4415, -4400, -4163, -3677, -2927, -1916,  -658,
          812,  2449,  4188,  5958,  7679,  9270, 10653, 11758, 12528, 12924, 12924, 12528,
        11758, 10653,  9270,  7679,  5958,  4188,  2449,   812,  -658, -1916, -2927, -3677,
        -4163, -4400, -4415, -4241, -3921, -3497, -3010, -2500, -2000, -1535, -1123,  -776,
         -499,  -287,  -137,   -39,    17,    44,    49,    43,    31,    18,     7,     0,
           -4,    -6,    -5,    -3,     0,     2,     4,     6,
    }}
}};

}} // mobilinkd::tnc

