// Read KISS-encoded AX.25 frames from a file and run them through the
// host-side digipeater test harness.

#include "test_digipeater_harness.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using test::TestDigipeater;
using mobilinkd::tnc::kiss::Alias;
using mobilinkd::tnc::kiss::NUMBER_OF_ALIASES;
using mobilinkd::tnc::kiss::hardware::ROUTING_SUBSTITUTE;
using test::ax25_packet_to_string;
using test::make_alias;
using test::make_test_config;

namespace {

constexpr uint8_t FEND = 0xC0;
constexpr uint8_t FESC = 0xDB;
constexpr uint8_t TFEND = 0xDC;
constexpr uint8_t TFESC = 0xDD;

struct Options {
    std::string mycall = "N0CALL";
    std::vector<Alias> aliases;
    std::string input_file;
};

[[noreturn]] void usage(const char* program, const std::string& error = {})
{
    if (!error.empty()) {
        std::cerr << "Error: " << error << "\n\n";
    }
    std::cerr << "Usage: " << program
              << " [--mycall CALL] [--alias NAME[,HOPS]]... <input_file>\n";
    std::exit(error.empty() ? 0 : 2);
}

std::string uppercase(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return value;
}

void validate_call(const std::string& call, const std::string& option)
{
    if (call.empty() || call.size() > 6) {
        throw std::runtime_error(option + " must contain 1 to 6 characters");
    }
    for (unsigned char c : call) {
        if (!std::isalnum(c)) {
            throw std::runtime_error(option + " must contain only letters and digits");
        }
    }
}

Alias parse_alias(const std::string& argument)
{
    std::string name = argument;
    unsigned long hops = 2;
    const auto comma = argument.find(',');
    if (comma != std::string::npos) {
        if (argument.find(',', comma + 1) != std::string::npos) {
            throw std::runtime_error("invalid alias: " + argument);
        }
        name = argument.substr(0, comma);
        const std::string hops_text = argument.substr(comma + 1);
        if (hops_text.empty()) {
            throw std::runtime_error("missing hop count in alias: " + argument);
        }
        size_t consumed = 0;
        try {
            hops = std::stoul(hops_text, &consumed);
        } catch (const std::exception&) {
            throw std::runtime_error("invalid hop count in alias: " + argument);
        }
        if (consumed != hops_text.size()) {
            throw std::runtime_error("invalid hop count in alias: " + argument);
        }
    }

    name = uppercase(name);
    validate_call(name, "alias");
    if (hops == 0 || hops > 15) {
        throw std::runtime_error("alias hop count must be between 1 and 15");
    }
    return make_alias(name, static_cast<uint8_t>(hops));
}

Options parse_options(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--help" || argument == "-h") {
            usage(argv[0]);
        } else if (argument == "--mycall") {
            if (++i >= argc) usage(argv[0], "--mycall requires a value");
            options.mycall = uppercase(argv[i]);
        } else if (argument == "--alias") {
            if (++i >= argc) usage(argv[0], "--alias requires a value");
            if (options.aliases.size() == NUMBER_OF_ALIASES) {
                usage(argv[0], "at most " + std::to_string(NUMBER_OF_ALIASES) +
                                   " aliases may be configured");
            }
            try {
                options.aliases.push_back(parse_alias(argv[i]));
            } catch (const std::exception& e) {
                usage(argv[0], e.what());
            }
        } else if (argument.rfind("--", 0) == 0) {
            usage(argv[0], "unknown option: " + argument);
        } else if (!options.input_file.empty()) {
            usage(argv[0], "only one input file may be specified");
        } else {
            options.input_file = argument;
        }
    }

    if (options.input_file.empty()) usage(argv[0], "missing input file");
    try {
        validate_call(options.mycall, "mycall");
    } catch (const std::exception& e) {
        usage(argv[0], e.what());
    }
    if (options.aliases.empty()) {
        options.aliases.push_back(make_alias("WIDE1", 2));
    }
    return options;
}

std::vector<uint8_t> read_binary_file(const std::string& path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("cannot open input file: " + path);
    }

    input.seekg(0, std::ios::end);
    const std::streamoff length = input.tellg();
    if (length < 0 || static_cast<unsigned long long>(length) >
                          std::numeric_limits<size_t>::max()) {
        throw std::runtime_error("cannot determine input file size: " + path);
    }
    input.seekg(0, std::ios::beg);

    std::vector<uint8_t> bytes(static_cast<size_t>(length));
    if (!bytes.empty() &&
        !input.read(reinterpret_cast<char*>(bytes.data()), length)) {
        throw std::runtime_error("cannot read input file: " + path);
    }
    return bytes;
}

bool kiss_unescape(const std::vector<uint8_t>& encoded,
                   std::vector<uint8_t>& decoded)
{
    decoded.clear();
    decoded.reserve(encoded.size());
    for (size_t i = 0; i < encoded.size(); ++i) {
        if (encoded[i] != FESC) {
            decoded.push_back(encoded[i]);
            continue;
        }
        if (++i == encoded.size()) return false;
        if (encoded[i] == TFEND) {
            decoded.push_back(FEND);
        } else if (encoded[i] == TFESC) {
            decoded.push_back(FESC);
        } else {
            return false;
        }
    }
    return true;
}

void process_kiss_frame(const std::vector<uint8_t>& encoded,
                        TestDigipeater& digipeater,
                        size_t& frame_number,
                        size_t& malformed_frames)
{
    std::vector<uint8_t> decoded;
    if (!kiss_unescape(encoded, decoded)) {
        ++malformed_frames;
        return;
    }
    if (decoded.empty() || decoded.front() != 0x00) {
        return; // Empty frames and non-data KISS commands are not AX.25 packets.
    }

    std::vector<uint8_t> ax25(decoded.begin() + 1, decoded.end());
    ++frame_number;

    std::cout << "--- Frame " << frame_number << " ---\n";
    std::cout << "Original:  " << ax25_packet_to_string(ax25, ax25.size()) << "\n";

    const Alias* alias = digipeater.can_repeat(ax25.data(), ax25.size());
    if (alias == nullptr) {
        std::cout << "  Routed:  NO\n";
        return;
    }

    std::array<uint8_t, TestDigipeater::LINEAR_BUF_SIZE> routed{};
    size_t routed_length = 0;
    if (!digipeater.rewrite_frame(ax25.data(), ax25.size(), routed.data(),
                                  routed_length, routed.size())) {
        ++malformed_frames;
        std::cout << "  Routed:  NO (rewrite failed)\n";
        return;
    }

    std::vector<uint8_t> routed_frame(routed.begin(), routed.begin() + routed_length);
    std::cout << "  Routed:  YES \xE2\x86\x92 "
              << ax25_packet_to_string(routed_frame, routed_frame.size()) << "\n";
}

} // namespace

int main(int argc, char** argv)
{
    const Options options = parse_options(argc, argv);

    try {
        const std::vector<uint8_t> input = read_binary_file(options.input_file);

        auto cfg = test::make_test_config(options.mycall, ROUTING_SUBSTITUTE);
        for (size_t i = 0; i < options.aliases.size() && i < NUMBER_OF_ALIASES; ++i) {
            cfg.aliases[i] = options.aliases[i];
        }
        TestDigipeater digipeater(cfg);

        std::vector<uint8_t> encoded_frame;
        bool inside_frame = false;
        size_t frame_number = 0;
        size_t malformed_frames = 0;

        for (uint8_t byte : input) {
            if (byte == FEND) {
                if (inside_frame && !encoded_frame.empty()) {
                    process_kiss_frame(encoded_frame, digipeater, frame_number,
                                       malformed_frames);
                }
                encoded_frame.clear();
                inside_frame = true;
            } else if (inside_frame) {
                encoded_frame.push_back(byte);
            }
        }

        if (inside_frame && !encoded_frame.empty()) {
            ++malformed_frames; // A KISS frame must end with FEND.
        }
        if (malformed_frames != 0) {
            std::cerr << "Warning: skipped " << malformed_frames
                      << " malformed KISS frame(s)\n";
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
