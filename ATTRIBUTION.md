# Attribution

## libaprsroute

This project's digipeater implementation was informed by libaprsroute, an APRS routing library by Ion Todirel.

- **Library:** libaprsroute
- **Version:** 0.1.0
- **Author:** Ion Todirel
- **Repository:** https://github.com/iontodirel/libaprsroute
- **License:** MIT

```
MIT License

Copyright (c) 2024-2025 Ion Todirel

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

### What we used

The TNC4 digipeater does not include libaprsroute source code. The routing is implemented directly on AX.25 7-byte address blocks in `Core/TNC/Digipeater.hpp` and `Core/TNC/Digipeater.cpp`.

However, the routing mode design was informed by libaprsroute:

- The routing mode flags (`ROUTING_PREEMPT_FRONT`, `ROUTING_PREEMPT_TRUNCATE`, `ROUTING_PREEMPT_DROP`, `ROUTING_PREEMPT_MARK`, `ROUTING_SUBSTITUTE`, `ROUTING_SKIP_COMPLETE`) correspond to libaprsroute's `routing_option` enum values (`preempt_front`, `preempt_truncate`, `preempt_drop`, `preempt_mark`, `substitute_complete_n_N_address`, `skip_complete_n_N_address`).
- The n-N routing algorithm (SSID hop decrement, H-bit marking, address matching by prefix) was validated against libaprsroute's test vectors.
- The recommended routing mode combination (`route_self | preempt_front | substitute_complete_n_N_address | trap_limit_exceeding_n_N_address | strict | preempt_n_N | substitute_explicit_address`) from libaprsroute informed our default mode selection.

See `docs/adr/0005-remove-libaprsroute-direct-buffer-routing.md` for the architecture decision to replace the library with direct buffer operations.
