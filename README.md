# libat

**High-performance C++23 header-only AT command parser**

Zero-copy, heap-allocation-free AT command parser compliant with
[ITU-T V.250](https://www.itu.int/rec/T-REC-V.250) and
[3GPP TS 27.007](https://www.3gpp.org/ftp/Specs/archive/27_series/27.007/).

---

## Features

| Feature | Detail |
|---|---|
| Standard | C++23 (`std::expected`, `std::generator`, `std::from_chars`) |
| Deployment | Header-only — single `#include <at/parser.hpp>` |
| Allocation | Zero heap allocations during parsing |
| Commands | Basic (ATH, ATA …), Extended (AT+CMD, AT&CMD …), S-parameters (ATS0=3) |
| Streaming | `at::stream_parser` for continuous serial-line input |
| Dispatch | `at::command_registry` with heterogeneous (zero-copy) lookup |
| Versioning | Git-aware semver via generated `<at/version.hpp>` |

---

## Requirements

- CMake ≥ 3.20
- C++23-capable compiler (GCC ≥ 14, Clang ≥ 17, MSVC ≥ 19.38)

---

## Quick start

### CMake FetchContent integration

```cmake
include(FetchContent)

FetchContent_Declare(
    libat
    GIT_REPOSITORY https://github.com/yanujz/libat.git
    GIT_TAG        v1.1.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(libat)

target_link_libraries(your_target PRIVATE at::libat)
```

### Parsing a single command

```cpp
#include <at/parser.hpp>
#include <iostream>

int main() {
    at::parser p{"AT+CGDCONT=1,\"IP\",\"internet\""};
    auto result = p.parse_command();

    if (result) {
        // result->name  == "CGDCONT"
        // result->type  == at::command_type::set
        auto apn = result->params.get_as<std::string_view>(2);
        std::cout << "APN: " << apn.value_or("") << '\n';  // APN: internet
    }
}
```

### Streaming serial input

```cpp
#include <at/parser.hpp>

at::stream_parser sp;

// Feed bytes as they arrive from UART; '\r' terminates each command.
auto cmds = sp.feed("AT+CMGF=1\rAT+CSQ\r");
if (cmds && !cmds->empty()) {
    for (const auto& cmd : *cmds) {
        // dispatch cmd ...
    }
}
```

---

## Project layout

```
libat/
├── CMakeLists.txt
├── include/
│   └── at/
│       ├── parser.hpp       # Library — include this
│       └── version.hpp.in   # CMake version template
├── examples/
│   └── basic_modem.cpp
└── tests/
    └── test_parser.cpp      # Google Test suite
```

---

## Building locally

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
./build/basic_modem
```

---

## License

MIT
