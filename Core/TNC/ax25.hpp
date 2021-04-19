// Copyright 2018 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__AX25_HPP_
#define MOBILINKD__TNC__AX25_HPP_

#include <array>

/**
 * This is starting to get complicated.  I'm not yet sure how to represent
 * the various forms of SSID for Source, Destination and Repeater.
 *
 * http://www.aprs.org/aprs12/precedence-bit.txt
 * http://www.aprs.org/aprs12/RR-bits.txt
 * http://www.aprs.org/aprs11/C-bits-SSID.txt
 * https://www.tapr.org/pdf/AX25.2.2.pdf
 */
struct SSID
{
    uint8_t continued : 1;
    uint8_t ssid : 4;
    uint8_t reserved_2 : 1;
    uint8_t reserved_1 : 1;
    union {
        uint8_t command : 1;
        uint8_t has_been_repeated : 1;
    };
};

struct AddressByte
{
    uint8_t continued : 1;
    uint8_t address : 7;
};

#define DEFAULT_ADDRESS_BYTE (' ' << 1)

struct Address
{
    static constexpr uint8_t
    typedef std::array<AddressByte, 6> address_type;

    address_type address{
        DEFAULT_ADDRESS_BYTE,
        DEFAULT_ADDRESS_BYTE,
        DEFAULT_ADDRESS_BYTE,
        DEFAULT_ADDRESS_BYTE,
        DEFAULT_ADDRESS_BYTE,
        DEFAULT_ADDRESS_BYTE};
    }
};

/**
 * In the destination address, the command bit is always set.
 */
struct DestinationAddress : Address
{

}

/**
 * In the source address, the command bit is always reset.
 */
struct SourceAddress : Address
{

}

/**
 * In the repeater address, the has_been_repeated bit indicates that
 * the the frame has been repeated.
 *
 *  > The H bit is set to “0” on frames going to a repeater.  The repeater
 *  > changes the H bit to “1” before it retransmits the frame.
 *  @ref https://www.tapr.org/pdf/AX25.2.2.pdf
 *
 *  Apparently for WIDE this is only set when N reaches 0.
 *
 *
 */
struct RepeaterAddress : Address
{

};

#endif // MOBILINKD__TNC__AX25_HPP_
