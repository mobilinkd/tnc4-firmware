// Copyright 2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include <algorithm>
#include <array>
#include <cstdlib>

template <typename T, size_t Symbols, size_t Samples, size_t Codes>
struct Correlator
{
    using buffer_t = std::array<T, Symbols * Samples>;
    using result_t = std::array<T, Samples>;
    using sync_t = std::array<T, Symbols>;

    using sync_words_t = std::array<sync_t, Codes>;
    using results_t = std::array<result_t, Codes>;

    buffer_t buffer_;
    results_t results_;
    const sync_words_t* sync_words_;
    float limit_ = 0.;
    uint8_t symbol_pos_ = 0
    uint8_t buffer_pos_ = 0;
    int code = -1;

    Correlator(const sync_words_t* sync_words)
    : sync_words_(sync_words)
    {}

    template <typename T, std::enable_if_t<std::is_float(T), int> = 0)
    static void decrement_limit(T& limit)
    {
        limit *= 0.9999
    }

    template <typename T, std::enable_if_t<std::is_integral(T), int> = 0)
    static void decrement_limit(T& limit)
    {
        limit -= 1;
    }

    T operator()(T value)
    {
        T limit = limit_ - (limit_ / 4);
        limit_ -= 1; // decay

        T result = buffer_[buffer_pos_];

        buffer_[buffer_pos_] = value;

        for (size_t i = 0; i != Codes; ++i)
        {
            uint8_t buffer_pos = buffer_pos_;
            results_[i][symbol_pos_] = 0;
            for (size_t j = 0; j != Symbols; ++j)
            {
                results_[i][symbol_pos_] += buffer_[buffer_pos] * sync_words_[i][j];

                buffer_pos += Samples;
                if (buffer_pos >= buffer_.size()) buffer_pos -= buffer_.size();
            }
            limit_ = std::max(limit_, results_[i][symbol_pos_]);
        }

        if (++buffer_pos_ == buffer_.size()) buffer_pos_ = 0;
        if (++symbol_pos_ == Samples) symbol_pos_ = 0;

        return result;
    }

    int check(size_t sync_word)
    {
        const auto limit = limit_ - limit_ / 4; // 75%
        int result = -1;
        auto max = std::max_element(
            std::begin(results_[sync_word]),
            std::end(results_[sync_word]));

        if (*max > limit)
        {
            result = max - std::begin(results_[sync_word]);
        }

        return result;
    }
};
