#!/usr/bin/env python3
"""Regenerate tests/test_routes_data.hpp from libaprsroute's routes.json.

Usage:
    python3 tests/gen_conformance_data.py [routes.json] > tests/test_routes_data.hpp

If no path is given, the script looks for the reference in the sibling
libaprsroute workspace (../libaprsroute/tests/routes.json) so the
regeneration works out of the box in the Mobilinkd development tree.

The reference routes.json models each route with three alias lists:

  - ``path``:               n-N aliases (e.g. "WIDE2", "WIDE2-1", "WIDE1").
                            Each entry is auto-classified by the reference:
                            a valid ``PREFIXn`` / ``PREFIXn-N`` form is n-N,
                            anything else (WIDE8, RELAY, plain WIDE) is
                            explicit.
  - ``explicit_addresses``: explicit aliases (full callsign, may carry an
                            SSID, e.g. "DIGI-7", "WIDE1-3").
  - ``n_N_addresses``:      n-N aliases.

This generator maps those to the firmware's class-mask model (ADR-0008):

  - n-N alias ``PREFIXn[-N]`` -> routing prefix ``PREFIX`` + class mask bit n.
    The optional hop limit N is dropped: the firmware serves N in 1..7 for
    any selected class, matching Direwolf's regex model.
  - explicit alias           -> full callsign, class mask 0.
"""

import json
import sys
import os

# routing_option name -> firmware routing_mode bit, matching KissTypes.hpp and
# the original generator.  Options missing from this table (reject/trap limits,
# route_self, strict) do not fit the 8-bit routing_mode and are marked
# unsupported.
OPTION_BITS = {
    "preempt_front": 0x01,
    "preempt_truncate": 0x02,
    "preempt_drop": 0x04,
    "preempt_mark": 0x08,
    "substitute_explicit_address": 0x10,
    "traceless_n_N_route": 0x20,
    "substitute_complete_n_N_address": 0x40,
    "skip_complete_n_N_address": 0x80,
}

MAX_ALIASES = 8


def parse_nn(addr):
    """Mirror libaprsroute's try_parse_n_N_address.

    Returns (prefix, class_mask) for a valid n-N address, or None if the
    address is not n-N (and should be treated as explicit).
    """
    text = addr[:-1] if addr.endswith("*") else addr
    sep = text.find("-")
    if sep == -1:
        if text and text[-1].isdigit():
            n = int(text[-1])
            if 1 <= n <= 7:
                return text[:-1], 1 << n
            return None
        return None
    # Exactly one digit on each side of the separator, e.g. WIDE2-1.
    if (
        sep >= 1
        and text[sep - 1].isdigit()
        and sep + 1 < len(text)
        and text[sep + 1].isdigit()
        and sep + 2 == len(text)
    ):
        n = int(text[sep - 1])
        N = int(text[sep + 1])
        if 1 <= n <= 7 and 0 <= N <= 7:
            return text[: sep - 1], 1 << n
        return None
    return None


def strip_ssid(addr):
    """Drop an explicit alias's -SSID suffix (the firmware matches the
    callsign; explicit SSID sensitivity is tracked separately)."""
    text = addr[:-1] if addr.endswith("*") else addr
    sep = text.find("-")
    return text[:sep] if sep != -1 else text


def map_options(options_str):
    bits = 0
    has_unsupported = False
    for opt in options_str.split(","):
        opt = opt.strip()
        if not opt:
            continue
        if opt in OPTION_BITS:
            bits |= OPTION_BITS[opt]
        else:
            has_unsupported = True
    return bits, has_unsupported


def collect_aliases(route):
    """Return an ordered list of (call, classes) aliases.

    n-N aliases are merged by prefix (their class masks OR'd); explicit
    aliases are deduplicated by callsign.  Order follows the reference:
    path (auto-classified), then explicit_addresses, then n_N_addresses.
    """
    aliases = []
    n_N_by_prefix = {}
    explicit_seen = set()

    def add_nn(prefix, mask):
        if prefix in n_N_by_prefix:
            n_N_by_prefix[prefix][1] |= mask
        else:
            n_N_by_prefix[prefix] = [prefix, mask]

    def add_explicit(call):
        if call not in explicit_seen:
            explicit_seen.add(call)
            aliases.append((call, 0))

    # path: auto-classified (n-N if valid, else explicit).
    for entry in (route.get("path") or "").split(","):
        entry = entry.strip()
        if not entry:
            continue
        parsed = parse_nn(entry)
        if parsed is not None:
            add_nn(parsed[0], parsed[1])
        else:
            add_explicit(strip_ssid(entry))

    for entry in (route.get("explicit_addresses") or "").split(","):
        entry = entry.strip()
        if entry:
            add_explicit(strip_ssid(entry))

    for entry in (route.get("n_N_addresses") or "").split(","):
        entry = entry.strip()
        if not entry:
            continue
        parsed = parse_nn(entry)
        if parsed is not None:
            add_nn(parsed[0], parsed[1])
        else:
            add_explicit(strip_ssid(entry))

    # Flatten: explicit aliases first (stable), then merged n-N aliases.
    result = list(aliases)
    for prefix, (_, mask) in n_N_by_prefix.items():
        result.append((prefix, mask))
    return result[:MAX_ALIASES]


def cstr(s):
    if s is None:
        return "nullptr"
    return json.dumps(s)  # json string escaping matches C++ string literals


def render_alias(call, classes):
    return "{%s, 0x%02x}" % (cstr(call), classes)


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        here = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(here, "..", "..", "libaprsroute", "tests", "routes.json")

    with open(path) as f:
        data = json.load(f)

    routes = data["routes"]

    # Drop comment-only entries (no test vector).
    routes = [r for r in routes if r.get("id")]

    out = []
    out.append("// AUTO-GENERATED by gen_conformance_data.py from libaprsroute/tests/routes.json")
    out.append("// DO NOT EDIT. Regenerate with: python3 tests/gen_conformance_data.py > tests/test_routes_data.hpp")
    out.append("//")
    out.append("// Alias schema (ADR-0008): `classes` is a class mask -- bit n (n=1..7)")
    out.append("// selects n-N class n; 0 marks an explicit (full-callsign) alias.")
    out.append("#pragma once")
    out.append("")
    out.append("#include <array>")
    out.append("#include <cstdint>")
    out.append("#include <cstring>")
    out.append("")
    out.append("struct AliasEntry { const char* call; uint8_t classes; };")
    out.append("")
    out.append("struct ConfRoute {")
    out.append("    const char* id;")
    out.append("    const char* address;")
    out.append("    const char* original_packet;")
    out.append("    const char* routed_packet;")
    out.append("    bool routed;")
    out.append("    uint8_t routing_mode;")
    out.append("    bool has_unsupported;")
    out.append("    const char* options_str;")
    out.append("    std::array<AliasEntry, %d> aliases;" % MAX_ALIASES)
    out.append("    uint8_t alias_count;")
    out.append("};")
    out.append("")
    out.append("static const std::array<ConfRoute, %d> CONF_ROUTES = {{" % len(routes))

    for r in routes:
        rid = r.get("id", "")
        address = r.get("address", "")
        original = r.get("original_packet", "")
        routed_pkt = r.get("routed_packet", "")
        routed = r.get("routed") == "true"
        options = r.get("options") or ""
        routing_mode, has_unsupported = map_options(options)
        aliases = collect_aliases(r)

        out.append("    {")
        out.append("        %s, %s, %s, %s," % (cstr(rid), cstr(address), cstr(original), cstr(routed_pkt)))
        out.append("        %s, 0x%02x," % ("true" if routed else "false", routing_mode))
        out.append("        %s, %s," % ("true" if has_unsupported else "false", cstr(options)))
        out.append("        {{")
        for i in range(MAX_ALIASES):
            if i < len(aliases):
                call, classes = aliases[i]
                entry = render_alias(call, classes)
            else:
                entry = "{nullptr, 0x00}"
            comma = "," if i < MAX_ALIASES - 1 else ""
            out.append("            %s%s" % (entry, comma))
        out.append("        }},")
        out.append("        %d" % len(aliases))
        out.append("    },")

    out.append("}};")
    out.append("")

    sys.stdout.write("\n".join(out))


if __name__ == "__main__":
    main()
