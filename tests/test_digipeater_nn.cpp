// APRS routing conformance tests for the 110 routes.json records without
// an options field. Test vectors adapted from libaprsroute routes.json,
// Copyright (c) 2024 Ion Todirel, licensed CC-BY 4.0.
#include "test_digipeater_harness.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
using namespace mobilinkd::tnc::kiss;
using namespace test;
static int passed = 0, failed = 0;
#define EXPECT_TRUE(c) do { if (!(c)) { std::cerr << "FAIL: " #c "\n"; failed++; return; } } while(0)
#define EXPECT_FALSE(c) EXPECT_TRUE(!(c))
#define TEST(name) void name(); struct name##_runner { name##_runner() { std::cout << "  " #name "... "; name(); } }; static name##_runner name##_instance; void name()
struct RouteCase { const char* id; const char* address; const char* alias; uint8_t hops; const char* original; const char* expected; bool routed; };
static const std::array<RouteCase, 110> routes = {{
    RouteCase{"41", "DIGI", "A", 1, "N0CALL>APRS,A,B,C,D,E,F,G:data", "N0CALL>APRS,DIGI,A*,B,C,D,E,F,G:data", true},
    RouteCase{"50", "DIGI", "CITYD", 1, "N0CALL>APRS,CITYA*,CITYB,CITYC,CITYD,CITYE:data", "N0CALL>APRS,CITYA*,CITYB,CITYC,CITYD,CITYE:data", false},
    RouteCase{"54", "DIGI", "WIDE1", 1, "N0CALL>DIGI,WIDE1-1:data", "N0CALL>DIGI,WIDE1-1:data", false},
    RouteCase{"55", "DIGI", "WIDE1", 1, "DIGI>N0CALL,WIDE1-1:data", "DIGI>N0CALL,WIDE1-1:data", false},
    RouteCase{"57", "DIGI", "WIDE1", 1, "FROM>APRS,CALL,WIDE1-1,DIGI*,WIDE2-1:data", "FROM>APRS,CALL,WIDE1-1,DIGI*,WIDE2-1:data", false},
    RouteCase{"58", "ROUTE", "DIGI", 1, "FROM>APRS,DIGI*,ROUTE,WIDE1-1,WIDE2-1:data", "FROM>APRS,DIGI*,ROUTE,WIDE1-1,WIDE2-1:data", false},
    RouteCase{"59", "ROUTE", "DIGI", 1, "FROM>APRS,DIGI,CALL*,ROUTE,WIDE1-1,WIDE2-1:data", "FROM>APRS,DIGI,CALL*,ROUTE,WIDE1-1,WIDE2-1:data", false},
    RouteCase{"60", "ROUTE", "DIGI", 1, "FROM>APRS,DIGI*,CALL,ROUTE,WIDE1-1,WIDE2-1:data", "FROM>APRS,DIGI*,CALL,ROUTE,WIDE1-1,WIDE2-1:data", false},
    RouteCase{"61", "ROUTE", "DIGI", 1, "FROM>APRS,DIGI,CALL,ROUTE*,WIDE1-1,WIDE2-1:data", "FROM>APRS,DIGI,CALL,ROUTE*,WIDE1-1,WIDE2-1:data", false},
    RouteCase{"62", "ROUTE", "DIGI", 1, "FROM>APRS,DIGI,ROUTE*,CALL,WIDE1-1,WIDE2-1:data", "FROM>APRS,DIGI,ROUTE*,CALL,WIDE1-1,WIDE2-1:data", false},
    RouteCase{"63", "DIGI", "WIDE1", 1, "DIGI>APRS,CALL,WIDE1,DIGI,WIDE2*:data", "DIGI>APRS,CALL,WIDE1,DIGI,WIDE2*:data", false},
    RouteCase{"64", "DIGI", "WIDE1", 1, "DIGI>APRS,CALL,WIDE1-1,WIDE2-2,DIGI,WIDE3*:data", "DIGI>APRS,CALL,WIDE1-1,WIDE2-2,DIGI,WIDE3*:data", false},
    RouteCase{"65", "DIGI", "WIDE1", 1, "DIGI>APRS,CALL,WIDE1-1,WIDE2-2,DIGI,WIDE3-2*:data", "DIGI>APRS,CALL,WIDE1-1,WIDE2-2,DIGI,WIDE3-2*:data", false},
    RouteCase{"66", "DIGI", "WIDE1", 1, "DIGI>APRS,CALLA,WIDE1-1,ROUTE*,WIDE2-2,DIGI,WIDE3-2,CALLB*:data", "DIGI>APRS,CALLA,WIDE1-1,ROUTE*,WIDE2-2,DIGI,WIDE3-2,CALLB*:data", false},
    RouteCase{"67", "DIGI", "WIDE2", 2, "N0CALL>APRS,WIDE2-2,DIGI:data", "N0CALL>APRS,WIDE2-2,DIGI:data", false},
    RouteCase{"68", "DIGI", "DIGI", 1, "N0CALL>APRS,CALL,DIGI:data", "N0CALL>APRS,CALL,DIGI:data", false},
    RouteCase{"69", "DIGI", "DIGI", 1, "N0CALL>APRS:data", "N0CALL>APRS:data", false},
    RouteCase{"70", "DIGI", "WIDE2", 1, "N0CALL>APRS,WIDE2-0:data", "N0CALL>APRS,WIDE2-0:data", false},
    RouteCase{"71", "DIGI", "WIDE8", 2, "N0CALL>APRS,WIDE8-2:data", "N0CALL>APRS,WIDE8-2:data", false},
    RouteCase{"72", "DIGI", "WIDE2", 1, "N0CALL>APRS,WIDE2:data", "N0CALL>APRS,WIDE2:data", false},
    RouteCase{"74", "DIGI", "CALL", 1, "N0CALL>APRS,CALL:data", "N0CALL>APRS,DIGI,CALL*:data", true},
    RouteCase{"75", "DIGI", "CALLB", 1, "N0CALL>APRS,CALLA*,CALLB,CALLC:data", "N0CALL>APRS,CALLA,DIGI,CALLB*,CALLC:data", true},
    RouteCase{"76", "DIGIA", "DIGIA", 1, "N0CALL>APRS,DIGIA,DIGIB,DIGIC,DIGID,DIGIE,DIGIF,DIGIG,DIGIH:data", "N0CALL>APRS,DIGIA*,DIGIB,DIGIC,DIGID,DIGIE,DIGIF,DIGIG,DIGIH:data", true},
    RouteCase{"77", "DIGIB", "DIGIB", 1, "N0CALL>APRS,DIGIA*,DIGIB,DIGIC,DIGID,DIGIE,DIGIF,DIGIG,DIGIH:data", "N0CALL>APRS,DIGIA,DIGIB*,DIGIC,DIGID,DIGIE,DIGIF,DIGIG,DIGIH:data", true},
    RouteCase{"78", "DIGIC", "DIGIC", 1, "N0CALL>APRS,DIGIA,DIGIB*,DIGIC,DIGID,DIGIE,DIGIF,DIGIG,DIGIH:data", "N0CALL>APRS,DIGIA,DIGIB,DIGIC*,DIGID,DIGIE,DIGIF,DIGIG,DIGIH:data", true},
    RouteCase{"79", "DIGID", "DIGID", 1, "N0CALL>APRS,DIGIA,DIGIB,DIGIC*,DIGID,DIGIE,DIGIF,DIGIG,DIGIH:data", "N0CALL>APRS,DIGIA,DIGIB,DIGIC,DIGID*,DIGIE,DIGIF,DIGIG,DIGIH:data", true},
    RouteCase{"80", "DIGIE", "DIGIE", 1, "N0CALL>APRS,DIGIA,DIGIB,DIGIC,DIGID*,DIGIE,DIGIF,DIGIG,DIGIH:data", "N0CALL>APRS,DIGIA,DIGIB,DIGIC,DIGID,DIGIE*,DIGIF,DIGIG,DIGIH:data", true},
    RouteCase{"81", "DIGIF", "DIGIF", 1, "N0CALL>APRS,DIGIA,DIGIB,DIGIC,DIGID,DIGIE*,DIGIF,DIGIG,DIGIH:data", "N0CALL>APRS,DIGIA,DIGIB,DIGIC,DIGID,DIGIE,DIGIF*,DIGIG,DIGIH:data", true},
    RouteCase{"82", "DIGIG", "DIGIG", 1, "N0CALL>APRS,DIGIA,DIGIB,DIGIC,DIGID,DIGIE,DIGIF*,DIGIG,DIGIH:data", "N0CALL>APRS,DIGIA,DIGIB,DIGIC,DIGID,DIGIE,DIGIF,DIGIG*,DIGIH:data", true},
    RouteCase{"83", "DIGIH", "DIGIH", 1, "N0CALL>APRS,DIGIA,DIGIB,DIGIC,DIGID,DIGIE,DIGIF,DIGIG*,DIGIH:data", "N0CALL>APRS,DIGIA,DIGIB,DIGIC,DIGID,DIGIE,DIGIF,DIGIG,DIGIH*:data", true},
    RouteCase{"84", "DIGI", "DIGI", 1, "N0CALL>APRS,DIGI,ROUTE:data", "N0CALL>APRS,DIGI*,ROUTE:data", true},
    RouteCase{"85", "DIGI", "DIGI", 1, "N0CALL>APRS,CALL*,DIGI:data", "N0CALL>APRS,CALL,DIGI*:data", true},
    RouteCase{"86", "DIGI", "DIGI", 1, "N0CALL>APRS,CALL,DIGI:data", "N0CALL>APRS,CALL,DIGI:data", false},
    RouteCase{"87", "DIGI", "WIDE1", 1, "N0CALL>APRS,DIGI,WIDE1-1:data", "N0CALL>APRS,DIGI*,WIDE1-1:data", true},
    RouteCase{"89", "DIGI", "DIGI", 1, "N0CALL>APRS,RELAY*,DIGI:data", "N0CALL>APRS,RELAY,DIGI*:data", true},
    RouteCase{"90", "ROUTER", "DIGI", 1, "DIGI>APRS,ROUTER:data", "DIGI>APRS,ROUTER*:data", true},
    RouteCase{"91", "ROUTER", "DIGI", 1, "DIGI>APRS,DIGI,ROUTER:data", "DIGI>APRS,ROUTER,DIGI*,ROUTER:data", true},
    RouteCase{"94", "ROUTER", "DIGI", 1, "DIGI>APRS,ROUTER,DIGI:data", "DIGI>APRS,ROUTER*,DIGI:data", true},
    RouteCase{"95", "ROUTER", "DIGI", 1, "DIGI>APRS,DIGI*,ROUTER:data", "DIGI>APRS,DIGI*,ROUTER:data", false},
    RouteCase{"96", "ROUTER", "DIGI", 1, "DIGI>APRS,ROUTER*,DIGI:data", "DIGI>APRS,ROUTER*,DIGI:data", false},
    RouteCase{"97", "ROUTER", "A", 1, "DIGI>APRS,A*,DIGI,ROUTER:data", "DIGI>APRS,A*,DIGI,ROUTER:data", false},
    RouteCase{"98", "ROUTER", "DIGI", 1, "DIGI>APRS,A*,DIGI,ROUTER:data", "DIGI>APRS,A*,DIGI,ROUTER:data", false},
    RouteCase{"102", "DIGI", "ROUTER", 1, "N0CALL>APRS,DIGI,ROUTER:data", "N0CALL>APRS,DIGI*,ROUTER:data", true},
    RouteCase{"103", "DIGI", "DIGI", 1, "N0CALL>APRS,WIDE,WIDE,WIDE:data", "N0CALL>APRS,DIGI,WIDE*,WIDE,WIDE:data", true},
    RouteCase{"105", "DIGI", "DIGI", 1, "N0CALL>APRS,WIDE*,WIDE,WIDE:data", "N0CALL>APRS,WIDE*,WIDE,WIDE:data", false},
    RouteCase{"107", "DIGI", "DIGI", 1, "N0CALL>APRS,DIGI,WIDE*,WIDE,WIDE:data", "N0CALL>APRS,DIGI,WIDE*,WIDE,WIDE:data", false},
    RouteCase{"108", "DIGI", "WIDE1", 2, "N0CALL>APRS,WIDE1-2:data", "N0CALL>APRS,DIGI*,WIDE1-1:data", true},
    RouteCase{"109", "DIGI", "WIDE1", 1, "N0CALL>APRS,WIDE1-0:data", "N0CALL>APRS,WIDE1-0:data", false},
    RouteCase{"110", "DIGI", "WIDE1", 2, "N0CALL>APRS,WIDE1-2:data", "N0CALL>APRS,DIGI*,WIDE1-1:data", true},
    RouteCase{"111", "DIGI", "WIDE1", 1, "N0CALL>APRS,WIDE1-1:data", "N0CALL>APRS,DIGI,WIDE1*:data", true},
    RouteCase{"113", "DIGI", "WIDE1", 2, "N0CALL>APRS,WIDE1-2:data", "N0CALL>APRS,DIGI*,WIDE1-1:data", true},
    RouteCase{"116", "DIGI", "RELAY", 1, "N0CALL>APRS,RELAY-1,WIDE1-1:data", "N0CALL>APRS,RELAY-1,WIDE1-1:data", false},
    RouteCase{"117", "DIGI", "RELAY", 1, "N0CALL>APRS,RELAY-1*,WIDE1-1:data", "N0CALL>APRS,RELAY-1,DIGI,WIDE1*:data", true},
    RouteCase{"120", "DIGI", "RELAY", 1, "N0CALL>APRS,RELAY,WIDE1-1:data", "N0CALL>APRS,DIGI,RELAY*,WIDE1-1:data", true},
    RouteCase{"121", "DIGI", "WIDE2", 1, "N0CALL>APRS,A,B,C,D,E,F*,WIDE2-1:data", "N0CALL>APRS,A,B,C,D,E,F,DIGI,WIDE2*:data", true},
    RouteCase{"122", "DIGI", "WIDE2", 1, "N0CALL>APRS,A,B,C,D,E,F,WIDE2-1:data", "N0CALL>APRS,A,B,C,D,E,F,WIDE2-1:data", false},
    RouteCase{"127", "DIGI", "WIDE2", 3, "N0CALL>APRS,WIDE2-3:data", "N0CALL>APRS,DIGI*,WIDE2-2:data", true},
    RouteCase{"133", "DIGI", "WIDE1", 3, "N0CALL>APRS,WIDE1-3:data", "N0CALL>APRS,DIGI*,WIDE1-2:data", true},
    RouteCase{"134", "ROUTE", "WIDE1", 2, "N0CALL>APRS,CALL*,WIDE1-2:data", "N0CALL>APRS,CALL,ROUTE*,WIDE1-1:data", true},
    RouteCase{"139", "REPEATER", "WIDE2", 1, "N0CALL>APRS,ROUTER,DIGI*,qAR,N0CALL:data", "N0CALL>APRS,ROUTER,DIGI*,qAR,N0CALL:data", false},
    RouteCase{"143", "DIGI", "WIDE1", 1, "N0CALL>APRS,WIDE3-3,CALL:data", "N0CALL>APRS,WIDE3-3,CALL:data", false},
    RouteCase{"144", "DIGI", "WIDE1", 1, "N0CALL>APRS,WIDE1-1,DIGI:data", "N0CALL>APRS,WIDE1-1,DIGI:data", false},
    RouteCase{"145", "DIGI", "WIDE1", 1, "N0CALL>APRS,WIDE1-1,DIGI:data", "N0CALL>APRS,WIDE1-1,DIGI:data", false},
    RouteCase{"146", "DIGI", "WIDE1", 1, "N0CALL>APRS,WIDE2-2,DIGI:data", "N0CALL>APRS,WIDE2-2,DIGI:data", false},
    RouteCase{"147", "DIGI", "WIDE1", 1, "N0CALL>APRS,DIGI,WIDE2-2:data", "N0CALL>APRS,DIGI*,WIDE2-2:data", true},
    RouteCase{"148", "DIGI", "WIDE3", 3, "N0CALL>APRS,CALL1,CALL2,CALL3,CALL4,CALL5,CALL6*,WIDE3-3:data", "N0CALL>APRS,CALL1,CALL2,CALL3,CALL4,CALL5,CALL6,DIGI*,WIDE3-2:data", true},
    RouteCase{"149", "DIGI", "WIDE3", 3, "N0CALL>APRS,CALL1,CALL2,CALL3,CALL4,CALL5,CALL6,CALL7*,WIDE3-3:data", "N0CALL>APRS,CALL1,CALL2,CALL3,CALL4,CALL5,CALL6,CALL7*,WIDE3-2:data", true},
    RouteCase{"150", "DIGI", "WIDE3", 1, "N0CALL>APRS,CALL1,CALL2,CALL3,CALL4,CALL5,CALL6,CALL7*,WIDE3-1:data", "N0CALL>APRS,CALL1,CALL2,CALL3,CALL4,CALL5,CALL6,CALL7,WIDE3*:data", true},
    RouteCase{"154", "DIGI", "WIDE3", 1, "N0CALL>APRS,WIDE3-1,A,B,C,D,E,F,G:data", "N0CALL>APRS,WIDE3*,A,B,C,D,E,F,G:data", true},
    RouteCase{"156", "CALLA", "WIDE1", 1, "N0CALL>APRS,WIDE2-2,CALLB:data", "N0CALL>APRS,WIDE2-2,CALLB:data", false},
    RouteCase{"157", "DIGI", "WIDE2", 2, "DIGI>APRS,WIDE2-2,DIGI:data", "DIGI>APRS,WIDE2-2,DIGI:data", false},
    RouteCase{"159", "DIGI", "WIDE1", 1, "N0CALL>APRS,WIDE1,WIDE2-2:data", "N0CALL>APRS,WIDE1,WIDE2-2:data", false},
    RouteCase{"167", "DIGI", "WIDE1", 1, "N0CALL>APRS,WIDE1,WIDE2-2:data", "N0CALL>APRS,WIDE1,WIDE2-2:data", false},
    RouteCase{"168", "DIGI", "WIDE1", 1, "N0CALL>APRS,WIDE1,WIDE2:data", "N0CALL>APRS,WIDE1,WIDE2:data", false},
    RouteCase{"169", "DIGI", "FOOBAR2", 2, "N0CALL>APRS,FOOBAR2-2:data", "N0CALL>APRS,DIGI*,FOOBAR2-1:data", true},
    RouteCase{"170", "DIGI", "FOOBAR2", 1, "N0CALL>APRS,FOOBAR2-1:data", "N0CALL>APRS,DIGI,FOOBAR2*:data", true},
    RouteCase{"193", "DIGI", "DIGI", 7, "N0CALL>APRS,DIGI-7:data", "N0CALL>APRS,DIGI,DIGI-7*:data", true},
    RouteCase{"195", "DIGI1", "DIGI1", 2, "N0CALL>APRS,DIGI1,DIGI1-2,DIGI1-3,DIGI2-1,DIGI2-2,DIGI2-3:data", "N0CALL>APRS,DIGI1-1,DIGI1*,DIGI1-2,DIGI1-3,DIGI2-1,DIGI2-2,DIGI2-3:data", true},
    RouteCase{"197", "DIGI", "DIGI", 1, "N0CALL>APRS,A,B,C,D,E,F,G,I:data", "N0CALL>APRS,DIGI*,B,C,D,E,F,G,I:data", true},
    RouteCase{"198", "DIGI", "DIGI", 1, "N0CALL>APRS,A,B,C,D,E,F,G:data", "N0CALL>APRS,DIGI,A*,B,C,D,E,F,G:data", true},
    RouteCase{"199", "DIGI1", "DIGI", 1, "N0CALL>APRS,DIGI,DIGI1-2,DIGI1-3,DIGI2-1,DIGI2-2,DIGI2-3:data", "N0CALL>APRS,DIGI1-1,DIGI*,DIGI1-2,DIGI1-3,DIGI2-1,DIGI2-2,DIGI2-3:data", true},
    RouteCase{"201", "DIGI1", "DIGI1", 3, "N0CALL>APRS,DIGI1-1,DIGI1-2,DIGI1-3,DIGI2-1,DIGI2-2,DIGI2-3:data", "N0CALL>APRS,DIGI1-1*,DIGI1-2,DIGI1-3,DIGI2-1,DIGI2-2,DIGI2-3:data", true},
    RouteCase{"202", "", "DIGI", 1, "N0CALL>APRS,DIGI,DIGI1-2,DIGI1-3,DIGI2-1,DIGI2-2,DIGI2-3:data", "N0CALL>APRS,DIGI,DIGI1-2,DIGI1-3,DIGI2-1,DIGI2-2,DIGI2-3:data", false},
    RouteCase{"203", "DIGI", "DIGI", 1, "N0CALL>APRS,DIGI,DIGI1-2,DIGI1-3,DIGI2-1,DIGI2-2,DIGI2-3:data", "N0CALL>APRS,DIGI*,DIGI1-2,DIGI1-3,DIGI2-1,DIGI2-2,DIGI2-3:data", true},
    RouteCase{"210", "DIGI", "DIGI", 1, "N0CALL>APRS,WIDE1-3:data", "N0CALL>APRS,DIGI,WIDE1-3*:data", true},
    RouteCase{"211", "DIGI", "DIGI", 1, "N0CALL>APRS,WIDE1-3:data", "N0CALL>APRS,WIDE1-3:data", false},
    RouteCase{"213", "DIGI", "DIGI", 7, "N0CALL>APRS,DIGI-7:data", "N0CALL>APRS,DIGI-7*:data", true},
    RouteCase{"215", "DIGI1", "DIGI", 1, "N0CALL>APRS,DIGI1-1:data", "N0CALL>APRS,DIGI1-1*:data", true},
    RouteCase{"218", "DIGI1", "DIGI", 1, "N0CALL>APRS,DIGI-1:data", "N0CALL>APRS,DIGI-1:data", false},
    RouteCase{"219", "DIGI1", "DIGI", 1, "N0CALL>APRS,DIGI1:data", "N0CALL>APRS,DIGI1:data", false},
    RouteCase{"220", "DIGI1", "DIGI", 1, "N0CALL>APRS,CALL*,DIGI1:data", "N0CALL>APRS,CALL*,DIGI1:data", false},
    RouteCase{"223", "DIGI", "WIDE1", 1, "N0CALL>APRS,WIDE1-1", "", false},
    RouteCase{"224", "DIGI", "WIDE1", 1, "APRS,WIDE1-1", "", false},
    RouteCase{"225", "DIGI", "WIDE1", 1, "N0CALL>APRS", "", false},
    RouteCase{"226", "DIGI", "WIDE1", 1, "N0CALL>APRS:", "", false},
    RouteCase{"227", "DIGI", "WIDE1", 1, "N0CALL>APRS,:", "", false},
    RouteCase{"228", "DIGI", "WIDE1", 1, "N0CALL", "", false},
    RouteCase{"233", "DIGI", "WIDE2", 2, "N0CALL>APRS,A,B,C,D,E,F,G*,WIDE2-2:data", "N0CALL>APRS,A,B,C,D,E,F,G*,WIDE2-1:data", true},
    RouteCase{"234", "DIGI", "WIDE2", 2, "N0CALL>APRS,A,B,C,D,E,F,G,WIDE2-2:data", "N0CALL>APRS,A,B,C,D,E,F,G,WIDE2-2:data", false},
    RouteCase{"235", "DIGI", "WIDE2", 2, "N0CALL>APRS,WIDE2-2,A,B,C,D,E,F,G:data", "N0CALL>APRS,WIDE2-1,A,B,C,D,E,F,G:data", true},
    RouteCase{"236", "DIGI", "WIDE2", 2, "N0CALL>APRS,A,B,C,D,E,F,G*,WIDE2-2:data", "N0CALL>APRS,A,B,C,D,E,F,G*,WIDE2-1:data", true},
    RouteCase{"239", "DIGI", "WIDE2", 1, "N0CALL>APRS,A,B,C,D,E,F,G*,WIDE2-1:data", "N0CALL>APRS,A,B,C,D,E,F,G,WIDE2*:data", true},
    RouteCase{"240", "DIGI", "WIDE2", 2, "N0CALL>APRS,A,B,C,D,E,F*,WIDE2-2:data", "N0CALL>APRS,A,B,C,D,E,F,DIGI*,WIDE2-1:data", true},
    RouteCase{"241", "DIGI", "WIDE2", 2, "N0CALL>APRS,A,B,C,D,E,F,WIDE2-2:data", "N0CALL>APRS,A,B,C,D,E,F,WIDE2-2:data", false},
    RouteCase{"251", "DIGI", "WIDE-", 1, "N0CALL>APRS,WIDE2-2", "", false},
    RouteCase{"252", "DIGI", "WIDE", 2, "N0CALL>APRS,WIDE2-2", "", false},
    RouteCase{"253", "DIGI", "WIDE2-", 1, "N0CALL>APRS,WIDE2-2", "", false},
    RouteCase{"254", "DIGI", "WIDE0", 2, "N0CALL>APRS,WIDE2-2", "", false},
    RouteCase{"255", "DIGI", "WIDE", 1, "N0CALL>APRS,WIDE2-2", "", false},
    RouteCase{"256", "DIGI", "WIDE2", 0, "N0CALL>APRS,WIDE2-2", "", false},
}};
static bool same(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) { return a == b; }
static bool run_route(const RouteCase& r) {
    auto cfg = make_test_config(r.address);
    if (*r.alias) cfg.aliases[0] = make_alias(r.alias, r.hops);
    TestDigipeater digi(cfg);
    auto in = parse_ax25_packet(r.original);
    auto match = in.empty() ? nullptr : digi.can_repeat(in.data(), in.size());
    if ((match != nullptr) != r.routed) {
        std::cerr << "route " << r.id << ": expected routed=" << r.routed
                  << ", got " << (match != nullptr) << " for " << r.original << "\n";
        return false;
    }
    if (!r.routed) return true;
    std::array<uint8_t, TestDigipeater::LINEAR_BUF_SIZE> out{}; size_t len = 0;
    if (!digi.rewrite_frame(in.data(), in.size(), out.data(), len, out.size())) {
        std::cerr << "route " << r.id << ": rewrite rejected frame\n"; return false;
    }
    std::vector<uint8_t> actual(out.begin(), out.begin() + len);
    auto expected = parse_ax25_packet(r.expected);
    if (!same(actual, expected)) {
        std::cerr << "route " << r.id << ": expected " << r.expected << ", got "
                  << ax25_packet_to_string(actual, actual.size()) << "\n"; return false;
    }
    return true;
}
TEST(routes_json_110_cases) {
    size_t route_failures = 0;
    for (const auto& r : routes) {
        if (!run_route(r)) {
            route_failures++;
        } else {
            passed++;
        }
    }
    if (route_failures != 0) {
        std::cerr << route_failures << " of " << routes.size()
                  << " routes.json cases failed\n";
        failed++;
    }
}
static bool rejected(const char* mycall, const char* alias, uint8_t hops, const char* packet) {
    auto cfg = make_test_config(mycall);
    cfg.aliases[0] = make_alias(alias, hops);
    TestDigipeater digi(cfg); auto in = parse_ax25_packet(packet);
    return digi.can_repeat(in.data(), in.size()) == nullptr;
}
TEST(edge_wide1_zero) { EXPECT_TRUE(rejected("DIGI", "WIDE1", 1, "N0CALL>APRS,WIDE1-0:data")); passed++; }
TEST(edge_wide1_no_ssid) { EXPECT_TRUE(rejected("DIGI", "WIDE1", 1, "N0CALL>APRS,WIDE1:data")); passed++; }
TEST(edge_hops_above_alias_limit) { EXPECT_TRUE(rejected("DIGI", "WIDE1", 1, "N0CALL>APRS,WIDE1-2:data")); passed++; }
TEST(edge_full_path_marks_without_insert) {
    auto cfg = make_test_config("DIGI", 0);
    cfg.aliases[0] = make_alias("WIDE2", 2); TestDigipeater digi(cfg);
    auto in = parse_ax25_packet("N0CALL>APRS,A,B,C,D,E,F,G*,WIDE2-2:data");
    EXPECT_TRUE(digi.can_repeat(in.data(), in.size()) != nullptr);
    std::array<uint8_t, TestDigipeater::LINEAR_BUF_SIZE> out{}; size_t len = 0;
    EXPECT_TRUE(digi.rewrite_frame(in.data(), in.size(), out.data(), len, out.size()));
    std::vector<uint8_t> actual(out.begin(), out.begin() + len);
    EXPECT_TRUE(same(actual, parse_ax25_packet("N0CALL>APRS,A,B,C,D,E,F,G*,WIDE2-1:data"))); passed++;
}
TEST(edge_our_destination) { EXPECT_TRUE(rejected("DIGI", "WIDE1", 1, "N0CALL>DIGI,WIDE1-1:data")); passed++; }
TEST(edge_source_is_mycall) { EXPECT_TRUE(rejected("DIGI", "WIDE1", 1, "DIGI>APRS,WIDE1-1:data")); passed++; }
TEST(edge_trace_relay_echo_gate) {
    for (const char* name : {"TRACE2", "RELAY", "ECHO2", "GATE2"}) {
        auto cfg = make_test_config("DIGI");
        cfg.aliases[0] = make_alias(name, 2); TestDigipeater digi(cfg);
        auto in = parse_ax25_packet(std::string("N0CALL>APRS,") + name + "-2:data");
        EXPECT_TRUE(digi.can_repeat(in.data(), in.size()) != nullptr);
    } passed++;
}
int main() {
    std::cout << "\nDigipeater n-N routing tests\n" << passed << " passed, " << failed << " failed\n";
    return failed ? 1 : 0;
}
