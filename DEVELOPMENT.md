# Development Guide

This document describes the architecture and internal design of libat.

## Architecture Overview

libat is a **header-only, zero-copy** AT command parser designed for embedded and real-time systems. It uses modern C++23 features to achieve high performance without heap allocations.

### Core Components

```
at::tokenizer           → Token-level parsing (lexer)
  ↓
at::parser              → Single-command parsing (produces at::command)
  ↓
at::stream_parser       → Streaming multi-command parsing with buffering
  ↓
at::command_registry    → Command dispatch (heterogeneous lookup, zero-copy)
```

## Module Guide

### `at::tokenizer` — Lexical Analysis

Converts a string into a stream of tokens (AT prefix, identifiers, operators, etc.).

**Key features:**
- Uses `token::type` enum for token classification
- C++23 generator coroutine for efficient streaming
- Zero allocations — works on string_view

**Files:** `include/at/parser.hpp` (lines ~335–504)

### `at::parser` — Single-Command Parser

Parses a single AT command string into an `at::command` object.

**Returns:** `std::expected<at::command, std::error_code>`

**Parses:**
- Basic commands: `ATZ`, `ATA`, `ATH`, etc.
- Extended commands: `AT+CMGF=1`, `AT+COPS?`, `AT+CGDCONT=?`
- S-parameters: `ATS0=3`, `ATS7?`

**Files:** `include/at/parser.hpp` (lines ~510–776)

### `at::stream_parser` — Streaming Parser

Maintains a `linear_buffer` and parses complete commands as they arrive in fragmented input.

**Key feature:** Handles serial/UART data where commands arrive in bursts separated by `\r` (S3=13 default).

**Important lifetime note:** 
String_view fields (`raw`, `prefix`, `name`) in returned `at::command` objects are invalidated after the buffer is partially erased. Safe pattern:
```cpp
at::stream_parser sp;
auto result = sp.feed("AT+CMGF=1\r");
if (result && !result->empty()) {
    std::string cmd_name(result->at(0).name);  // COPY immediately!
    // ... safe to use cmd_name after next feed()
}
```

**Files:** `include/at/parser.hpp` (lines ~843–967)

### `at::parameters` — Parameter Storage & Retrieval

Stores parsed command parameters with type information.

**Supports:**
- Integers (`int64_t`)
- Floats (`double`)
- Strings (`std::string_view`)
- Booleans (`bool`)
- Optional values (`std::monostate`)

**Zero-allocation design:**
- Uses `from_chars` (C++17+) instead of `std::stoi` / `std::stod`
- Lazy parsing only when `.get_as<T>()` is called

**Files:** `include/at/parser.hpp` (lines ~163–271)

### `at::command_registry` — Command Dispatch

Maps command names to handler functions with heterogeneous (zero-copy) lookup.

**Example:**
```cpp
at::command_registry reg;
reg.register_handler("CMGF", [](const at::command& cmd) {
    if (auto val = cmd.params.get_as<int64_t>(0)) {
        // Handle CMGF mode change
    }
});

reg.dispatch(parsed_cmd);  // Zero-copy lookup!
```

**Files:** `include/at/parser.hpp` (lines ~973–1013)

## Standards Compliance

### ITU-T V.250 (General Requirements for Terminals)

- **Section 5.2:** Command format (AT-prefix, basic/extended commands)
- **Section 5.3:** Response format (OK, ERROR, CONNECT, RING, etc.)
- **Section 5.4:** Extended commands (AT+ syntax)
- **Section 6:** S-parameters and registers

### 3GPP TS 27.007 (AT Command Set)

- Standard modem AT commands for GSM/LTE devices
- AT+CMGF, AT+CMGS, AT+CREG, AT+CGDCONT, etc.
- Error codes: +CME ERROR, +CMS ERROR

All test cases use real-world, standards-compliant examples.

## Memory Efficiency

### Zero-Copy Guarantees

1. **No dynamic allocation in parsing** — uses `std::string_view` throughout
2. **Stack-only parameters** — `parameters` is stack-resident
3. **No string concatenation** — views into input buffer
4. **No temporary vectors** (except in tests/examples)

### Buffer Management

The `linear_buffer` class in `stream_parser` uses:
- A `std::vector<char>` for resizable storage
- `erase()` to drop consumed bytes (necessary for stateful parsing)
- No ring-buffer complexity — simpler for embedded use

## Testing Strategy

### Test Coverage

**36 tests organized into 5 suites:**
1. `ParserBasicCommands` — single-letter commands
2. `ParserExtendedCommands` — AT+CMD variations
3. `ParserSParameters` — ATS-register commands
4. `ParserErrorCases` — malformed input handling
5. `StreamParser` — fragmented and multi-command input

### Test Compliance

Every test string is **100% compliant** with ITU-T V.250 / 3GPP TS 27.007:
- Real modem commands (not made-up)
- Proper quoting, commas, parameters
- S-register ranges and default values

**Files:** `tests/test_parser.cpp`

## Performance Considerations

### Why Zero-Copy Matters

In embedded systems (modems, IoT):
- Heap allocations are slow and fragmentation-prone
- Real-time constraints demand predictable performance
- `std::string_view` allows parsing without copying

### Measured Characteristics

- Single command parse: ~microseconds (stack-only)
- Fragmented reassembly: minimal overhead (linear buffer erase)
- No lock contention (single-threaded design intended for UART ISR)

## Future Enhancements

Possible improvements without breaking the API:

1. **Ring buffer** for stream_parser (avoid erase cost)
2. **Custom allocator** support for embedded systems with restricted heaps
3. **Response parsing** improvements (currently basic)
4. **AT command builder** utilities (class `command_builder` is minimal)
5. **UART integration examples** (non-blocking stream handling)

## Debugging Tips

### GDB Breakpoints

```gdb
# Break on parser error
b at::parser::parse_command_body

# Break on stream completion
b at::stream_parser::try_parse_one
```

### Common Issues

**Issue:** `string_view` pointing to freed data after `stream_parser::feed()`
**Solution:** Copy the name/raw immediately: `std::string name(cmd.name);`

**Issue:** S-parameter index not parsed
**Solution:** Check if `cmd.s_index.has_value()` — not set for non-S commands

**Issue:** Float parameter returns `std::nullopt`
**Solution:** Verify it was parsed as double, not int; check parameter format

## References

- [C++23 Reference](https://en.cppreference.com/w/cpp)
- [ITU-T V.250 Recommendation](https://www.itu.int/rec/T-REC-V.250)
- [3GPP TS 27.007 (AT commands)](https://www.3gpp.org/ftp/Specs/archive/27_series/27.007/)
- [Google Test Documentation](https://google.github.io/googletest/)
