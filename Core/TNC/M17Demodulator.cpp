// Copyright 2020-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "M17Demodulator.h"
#include "AudioLevel.hpp"

#include "main.h"

#include "stm32l4xx_hal.h"

#include <array>
#include <cstdint>

namespace mobilinkd { namespace tnc {

constexpr uint8_t MAX_MISSING_SYNC = 5;

m17::Indicator lsf_indicator{GPIOB, GPIO_PIN_0};
m17::Indicator dcd_indicator{GPIOA, GPIO_PIN_7};
m17::Indicator str_indicator{GPIOA, GPIO_PIN_6};

void M17Demodulator::start()
{
    SysClock72();

    demod_filter.init(m17::rrc_taps_f15.data());
    passall(kiss::settings().options & KISS_OPTION_PASSALL);
    audio::virtual_ground = (VREF + 1) / 2;

//    hadc1.Init.OversamplingMode = DISABLE;
//    if (HAL_ADC_Init(&hadc1) != HAL_OK)
//    {
//        CxxErrorHandler();
//    }

    ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel = AUDIO_IN;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        CxxErrorHandler();

    startADC(2499, ADC_BLOCK_SIZE);
//    getModulator().start_loopback();
    dcd_off();
}

void M17Demodulator::update_values(uint8_t index)
{
	m17::correlator.apply([this,index](float t){dev.sample(t);}, index);
	dev.update();
	sample_index = index;
}

void M17Demodulator::dcd_on()
{
	// Data carrier newly detected.
	dcd_ = true;
	dcd_indicator.on();
	sync_count = 0;
	dev.reset();
    framer.reset();
    decoder.reset();
	missing_sync_count = 0;
}

void M17Demodulator::dcd_off()
{
	// Just lost data carrier.
	dcd_ = false;
	dcd_indicator.off();
	demodState = DemodState::UNLOCKED;
}

hdlc::IoFrame* M17Demodulator::operator()(const q15_t* input)
{
	str_indicator.on();
	static bool need_clock_reset = false;
	static bool need_clock_update = false;

    hdlc::IoFrame* frame_result = nullptr;

    const float inv = 1.0 / 2048.0;

    if (!dcd_ && dcd.dcd())
    {
    	dcd_on();
    	need_clock_reset = true;
    }
    else if (dcd_ && !dcd.dcd())
    {
    	dcd_off();
    }

    auto filtered = demod_filter(input, 1.f / 2560.f);
//    getModulator().loopback(filtered);

    uint8_t pre_index;
    uint8_t lsf_index;
    uint8_t pkt_index = 0;

    int8_t pre_updated = 0;
    int8_t lsf_updated = 0;
    int8_t pkt_updated = 0;

    for (size_t i = 0; i != ADC_BLOCK_SIZE; ++i)
    {
        auto filtered_sample = filtered[i];
        m17::correlator.sample(filtered_sample);
    	dcd(input[i] * inv);
        if (!dcd_) continue; // No need to go further if no DCD.

        if (m17::correlator.index() == 0)
        {
			if (need_clock_reset)
			{
				clock_recovery.reset();
				need_clock_reset = false;
			}
			if (need_clock_update)
			{
				clock_recovery.update();
				auto idx = clock_recovery.sample_index();
				if (abs(idx - sample_index) <= 1) sample_index = idx;
				need_clock_update = false;
			}
        }

    	clock_recovery(filtered_sample);

		if (demodState != DemodState::UNLOCKED && m17::correlator.index() == sample_index)
		{
			dev.sample(filtered_sample);
		}

		float triggered;

        switch (demodState)
        {
        case DemodState::UNLOCKED:
        	// In this state, the sample_index is unknown.  We need to find
        	// a sync word to find the proper sample_index.  We only leave
        	// this state if we believe that we have a valid sample_index.

			pre_index = m17::preamble_sync(m17::correlator);
			pre_updated = m17::preamble_sync.updated();
			if (pre_updated)
			{
				sync_count = 0;
				need_clock_reset = true;
				dev.reset();
				update_values(pre_index);
				ITM_SendChar('.');
				demodState = DemodState::LSF_SYNC;
				INFO("p sync %d", pre_index);
				break;
			}
			if (missing_sync_count > 1920)
			{
				ITM_SendChar('*');
				need_clock_update = true;
				demodState = DemodState::STREAM_SYNC;
			}
			missing_sync_count += 1;
			break;
        case DemodState::LSF_SYNC:
        	if (m17::correlator.index() == sample_index)
        	{
            	triggered = m17::preamble_sync.triggered(m17::correlator);
				if (triggered != 0)
				{
					update_values(sample_index);
					ITM_SendChar('.');
					break;
				}
				triggered = m17::lsf_sync.triggered(m17::correlator);
				if (triggered != 0)
				{
					missing_sync_count = 0;
					need_clock_update = true;
					update_values(sample_index);
					if (triggered > 0)
					{
						demodState = DemodState::FRAME;
						sync_word_type = M17FrameDecoder::SyncWordType::LSF;
						INFO("l sync %d", int(sample_index));
					}
					else
					{
						demodState = DemodState::FRAME;
						sync_word_type = M17FrameDecoder::SyncWordType::STREAM;
						INFO("s sync %d", int(sample_index));
					}
				}
				else if (++sync_count > 192)
				{
					demodState = DemodState::UNLOCKED;
					INFO("l unlock %d", int(sync_count));
				}
				else
				{
					update_values(sample_index);
				}
        	}
            break;
        case DemodState::STREAM_SYNC:
			lsf_index = m17::lsf_sync(m17::correlator);
			lsf_updated = m17::lsf_sync.updated();
			sync_count += 1;
			if (lsf_updated < 0)
			{
				missing_sync_count = 0;
				if (sync_count <= 70)
				{
					update_values(lsf_index);
					sync_word_type = M17FrameDecoder::SyncWordType::STREAM;
					demodState = DemodState::FRAME;
					INFO("s early");
				}
				else if (sync_count > 70)
				{
					update_values(lsf_index);
					sync_word_type = M17FrameDecoder::SyncWordType::STREAM;
					demodState = DemodState::FRAME;
					INFO("s sync %d", int(lsf_index));
				}
				break;
			}
            else if (sync_count > 97)
            {
				dev.update();
            	missing_sync_count += 1;
            	if (missing_sync_count < MAX_MISSING_SYNC)
            	{
					sync_word_type = M17FrameDecoder::SyncWordType::STREAM;
					demodState = DemodState::FRAME;
					INFO("s unsync %d", int(missing_sync_count));
            	}
            	else
            	{
					demodState = DemodState::LSF_SYNC;
					INFO("s unlock");
            	}
            }
            break;
        case DemodState::PACKET_SYNC:
			pkt_index = m17::packet_sync(m17::correlator);
			pkt_updated = m17::packet_sync.updated();
			sync_count += 1;
			if (sync_count > 70 && pkt_updated)
			{
				missing_sync_count = 0;
				update_values(pkt_index);
				sync_word_type = M17FrameDecoder::SyncWordType::PACKET;
				demodState = DemodState::FRAME;
				INFO("k sync");
				break;
			}
			else if (sync_count > 98)
			{
				missing_sync_count += 1;
            	if (missing_sync_count < MAX_MISSING_SYNC)
            	{
					sync_word_type = M17FrameDecoder::SyncWordType::PACKET;
					demodState = DemodState::FRAME;
					INFO("k unsync");
            	}
            	else
            	{
					demodState = DemodState::UNLOCKED;
					INFO("k unlock");
            	}
			}
			break;
        case DemodState::FRAME:
			if (m17::correlator.index() == sample_index)
			{
				auto demod_result = demod(filtered_sample);
				auto [sample, phase, symbol, evm] = demod_result;

				auto n = llr<float, 4>(sample);
				int8_t* tmp;
				auto len = framer(n, &tmp);
				if (len != 0)
				{
					need_clock_update = true;

					std::copy(tmp, tmp + len, buffer.begin());
					auto valid = decoder(sync_word_type, buffer, frame_result, ber);
					INFO("demod: %d, dt: %7d, evma: %5d, dev: %5d, freq: %5d, locked: %d, ber: %d",
						int(decoder.state()), int(clock_recovery.clock_estimate() * 1000000),
						int(symbol_evm.evm() * 1000), int(dev.deviation() * 1000),
						int(dev.offset() * 1000),
						int(clock_recovery.sample_index()), ber);

					switch (decoder.state())
					{
					case M17FrameDecoder::State::STREAM:
						lsf_indicator.off();
						demodState = DemodState::STREAM_SYNC;
						break;
					case M17FrameDecoder::State::LSF:
						// If state == LSF, we need to recover LSF from LICH.
						lsf_indicator.on();
						demodState = DemodState::STREAM_SYNC;
						break;
					default:
						demodState = DemodState::PACKET_SYNC;
						break;
					}

					sync_count = 0;

					switch (valid)
					{
					case M17FrameDecoder::DecodeResult::FAIL:
						WARN("decode invalid");
						if (frame_result && !passall_)
						{
						   if (frame_result) hdlc::release(frame_result);
						   frame_result = nullptr;
						}
						break;
					case M17FrameDecoder::DecodeResult::EOS:
						demodState = DemodState::LSF_SYNC;
						INFO("EOS");
						break;
					case M17FrameDecoder::DecodeResult::OK:
						// INFO("valid frame for sw %d", int(sync_word_type));
						break;
					case M17FrameDecoder::DecodeResult::INCOMPLETE:
						// INFO("lich frame for sw %d", int(sync_word_type));
						break;
					}
				}
			}
			break;
        }
    }
    str_indicator.off();
//    INFO("dcd = %d", int(dcd.level() * 1000));

    return frame_result;
}

M17Demodulator::demod_result_t M17Demodulator::demod(float sample)
{
	sample -= dev.offset();
	sample *= dev.idev();
    auto [symbol, evm] = symbol_evm(sample);
    evm_average = symbol_evm.evm();
    float polarity = kiss::settings().rx_rev_polarity() ? -1.f : 1.f;

    return std::make_tuple(sample, 0.f, symbol * polarity, evm);
}

}} // mobilinkd::tnc
