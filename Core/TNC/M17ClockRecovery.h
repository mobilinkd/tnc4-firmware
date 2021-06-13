// Copyright 2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "IirFilter.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>

namespace mobilinkd { namespace m17 {

// 1/16
//constexpr std::array<float, 3> loop_b = { 0.00844269,  0.01688539,  0.00844269 };
//constexpr std::array<float, 3> loop_a = { 1.        , -1.72377617,  0.75754694 };

// 1/32
constexpr std::array<float, 3> lock_b = { 0.00225158, 0.00450317, 0.00225158 };
constexpr std::array<float, 3> lock_a = { 1.        , -1.86136115,  0.87036748 };

// 1/64 4-pole
//constexpr std::array<float, 5> lock_b = { 3.40605298e-07, 1.36242119e-06, 2.04363179e-06,
//		1.36242119e-06, 3.40605298e-07 };
//constexpr std::array<float, 5> lock_a = { 1.        , -3.87173472,  5.62336619, -3.63122726,
//		0.87960124 };

// 1/4
constexpr std::array<float,3> loop_b = {0.09763107,  0.19526215,  0.09763107};
constexpr std::array<float,3> loop_a = {1.        , -0.94280904,  0.33333333};

/**
 * M17 4-FSK clock recovery.
 *
 * Estimates the difference between the local clock and the remote clock using
 * two inputs: the symbol timing estimate from the correlator and the symbol
 * phase estimator.
 *
 * The correlator provides a strong indication of the preferred sample index
 * every 40ms.  The symbol phase estimator provides a very rough and noisy
 * estimate at every symbol, about every 200us.
 *
 * The maximum clock error rate allowed by this implementation is 500ppm, a
 * drift of up to 24 samples per second, or just under 1 symbol per frame.
 * This allows each clock to be off by up to 250ppm.  This was chosen because
 * this is the error rate allowed by USB, so many microcontrollers will have
 * a clock that is at least this accurate.
 *
 * Reducing this maximum error rate allows the gain to be increased resulting
 * in faster convergence.
 *
 * The estimated clock rate will never be outside of the bounds of the max
 * error rate allowed.
 *
 * The clock recovery system uses noise from the correlator to determine whether
 * a signal is present.  In the presense of noise, this will average around 3-4
 * samples and will have no time correlation.
 *
 * A preamble detection will have a time correlation of 2 symbols.
 *
 * A link setup frame detection will have a time correlation of 8 symbols.
 *
 * A data frame detection will have a time correlation of 192 symbols.
 *
 * Spurious detection of strong correlator output during data reception
 * is possible.  Testing so far indicates that these should be ignored.
 *
 * The phase estimator can be used to converge to a very accurate clock
 * estimation over time.  The goal is to estimate the symbol tick rate,
 * which will be between 1.0005 and 0.9995.
 */
struct ClockRecovery
{
	tnc::IirFilter<3> lock_filter{lock_b, lock_a};
	tnc::IirFilter<3> loop_filter{loop_b, loop_a};

	static constexpr float MAX_RATE = 1.0005f;
	static constexpr float MIN_RATE = 0.9995f;

	float filtered_index;
	float jitter;
	float lock_level;
	float unlock_level;
	bool locked = false;
	const int8_t sps;
	const int8_t max_delta;
	int8_t last_index;
	uint32_t estimator_count = 0;
	float clock = 0.f;

	ClockRecovery(float lock, float unlock, int8_t sps = 10)
	: lock_level(lock), unlock_level(unlock), sps(sps), max_delta(sps / 2), last_index(max_delta)
	{
		for (size_t i = 0; i != 10; ++i) lock_filter(1.f);
	}

	int8_t tick() {return 0;}

	void unlock() { lock_filter(3); }

	bool is_locked()
	{
		if (locked && jitter > unlock_level)
		{
			locked = false;
		}
		else if (!locked && jitter < lock_level)
		{
			locked = true;
		}
		return locked;
	}

	int8_t timing_delta(int8_t timing_index)
	{
		int8_t delta = timing_index - last_index;
		if (delta > max_delta)
		{
			delta -= sps;
		}
		else if (delta < -max_delta)
		{
			delta += sps;
		}
		last_index = timing_index;
		return delta;
	}

	auto operator()(int8_t timing_index)
	{
		// This is complicated because the timing index has a discontinuity.
		// The only way around this is to normalize the index in some manner.
		// The best way to do this is to track the difference from one
		// estimate to the next, so we measure the delta.

		auto delta = timing_delta(timing_index);
		auto filtered_index = loop_filter(timing_index + delta);

		INFO("dt = %d, fi = %d", int(delta), int(std::roundf(filtered_index)));

		jitter = lock_filter(std::abs(delta));
		int8_t estimated_index = int(std::roundf(filtered_index)) % sps;

		return std::make_tuple(estimated_index, is_locked());
	}

	/**
	 * Adjust the sampling index based on the output of the correlator.
	 *
	 * This adjust the rough timing estimate based on the output of the
	 * correlator, using its estimate of the best sample point.
	 */
	auto correlator(int8_t timing_index)
	{
		// This is complicated because the timing index has a discontinuity.
		// The only way around this is to normalize the index in some manner.
		// The best way to do this is to track the difference from one
		// estimate to the next, so we measure the delta.

		auto delta = timing_delta(timing_index);
		auto filtered_index = loop_filter(timing_index + delta);

		INFO("dt = %d, fi = %d", int(delta), int(std::roundf(filtered_index)));

		jitter = lock_filter(std::abs(delta));
		int8_t estimated_index = int(std::roundf(filtered_index)) % sps;

		return std::make_tuple(estimated_index, is_locked());
	}

	/**
	 * Adjust timing information based on symbol phase estimate.
	 *
	 * This function adjust the clock estimate based on received symbol phase
	 * estimates.  The symbol timing estimate narrows when locked based on the
	 * duration of the lock.
	 *
	 * What we should see is that the phase estimate wobbles a bit as the
	 * sample point drifts from sample to the next.  If the clocks are very
	 * closely aligned, the happens rarely, once per second at 20ppm, but
	 * can happen 10 times a seconds at 200ppm difference.
	 *
	 * If both clocks are relatively stable, and most clocks are, then the
	 * estimate just more and more refined over time.
	 *
	 * Using phase estimation, the timing converges well, even when the channel
	 * is rather noisy, since we are integrating over a long duration.
	 */
	auto phase(float phase)
	{
//		auto delta = timing_delta(timing_index);
//		auto filtered_index = loop_filter(timing_index + delta);
//
//		INFO("dt = %d, fi = %d", int(delta), int(std::roundf(filtered_index)));
//
//		jitter = lock_filter(std::abs(delta));
//		int8_t estimated_index = int(std::roundf(filtered_index)) % sps;
//
//		return std::make_tuple(estimated_index, is_locked());
	}
};

}} // mobilinkd::m17
