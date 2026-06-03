# Three device variants, three repos — but working toward unification

The TNC3, TNC4, and NucleoTNC firmware lives in three separate git repositories. Changes that apply to multiple devices are manually diffed and applied across repos. `#ifdef` guards handle device differences in higher-level code; most of `Core/TNC/` is portable.

We decided to keep the three repos for now — each device has real hardware differences (MCU variant, pin count, peripheral availability) and a single unified codebase isn't feasible yet. But we're working toward it incrementally: refactoring `Core/TNC/` outside of `Core/`, extracting pure-abstract interfaces for modulators and demodulators, and making hardware-agnostic components testable.

## Considered Options

**Single unified repo with `#ifdef` everything.** Rejected — the codebases have diverged enough that this would be a high-risk rewrite. `#ifdef` sprawl would make the code harder to read, not easier.

**Keep three repos, no unification effort.** Rejected — the maintenance burden of manual diff-and-apply is growing. Each fix or feature requires 3x the work.

**Incremental unification via extracted portable modules.** Selected — pull hardware-agnostic code into shared libraries or submodules one piece at a time. Start with the frame pool, SLIP codec, and KissHardware configuration logic. This reduces risk and delivers value incrementally.

## Consequences

- Portable modules must be designed for testability — mocking STM32/CMSIS hardware is impractical, so hardware-agnostic interfaces are a prerequisite
- The `hdlc::Frame` pool, SLIP encoders/decoders, and `KissHardware` EEPROM-backed configuration are the most promising candidates for early extraction and unit testing
- `Core/TNC/` needs to move outside `Core/` before meaningful module extraction can begin
