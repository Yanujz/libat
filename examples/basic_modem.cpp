/**
 * @file basic_modem.cpp
 * @brief libat example — simulating an AT command server for a cellular modem.
 *
 * Demonstrates:
 *   1. Printing the library version from the generated version.hpp.
 *   2. Parsing individual AT commands with at::parser.
 *   3. Using at::stream_parser to handle a continuous serial-data stream,
 *      including commands arriving in multiple fragments.
 *
 * All command strings are 100 % compliant with ITU-T V.250 / 3GPP TS 27.007.
 * The S3 register default (13 = CR) is used as the line terminator for the
 * stream parser, so every command is terminated with '\r'.
 */

#include <at/parser.hpp>
#include <at/version.hpp>

#include <array>
#include <format>
#include <iostream>
#include <string>
#include <string_view>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static std::string_view command_type_name(at::command_type t) noexcept {
    switch (t) {
        case at::command_type::basic:       return "basic";
        case at::command_type::extended:    return "extended/execute";
        case at::command_type::read:        return "read";
        case at::command_type::test:        return "test";
        case at::command_type::set:         return "set";
        case at::command_type::s_parameter: return "s-parameter";
        default:                            return "unknown";
    }
}

/// Print a single parsed command to stdout.
static void print_command(const at::command& cmd) {
    std::cout << std::format("  name={:<12} type={:<20} s_index={}",
        cmd.name,
        command_type_name(cmd.type),
        cmd.s_index ? std::to_string(*cmd.s_index) : "-");

    if (!cmd.params.empty()) {
        std::cout << "  params=[";
        bool first = true;
        for (const auto& [key, val] : cmd.params) {
            if (!first) std::cout << ", ";
            first = false;
            std::visit([](const auto& v) {
                using T = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<T, std::monostate>)
                    std::cout << "<empty>";
                else if constexpr (std::is_same_v<T, std::string_view>)
                    std::cout << std::format("\"{}\"", v);
                else if constexpr (std::is_same_v<T, bool>)
                    std::cout << (v ? "true" : "false");
                else
                    std::cout << v;
            }, val);
        }
        std::cout << "]";
    }
    std::cout << '\n';
}

// ---------------------------------------------------------------------------
// Part 1 — at::parser: parse individual command strings
// ---------------------------------------------------------------------------

static void demo_direct_parser() {
    std::cout << "\n── Direct parser (at::parser) ──────────────────────────────\n";

    // A realistic command sequence a DTE might send to initialise a modem.
    const std::array commands{
        std::string_view{"ATZ"},                          // soft reset
        std::string_view{"ATE"},                          // echo mode
        std::string_view{"ATI"},                          // request identification
        std::string_view{"AT+CMGF=1"},                   // SMS text mode
        std::string_view{"AT+CREG=2"},                   // enable network registration URC
        std::string_view{"AT+CGDCONT=1,\"IP\",\"internet\""}, // define PDP context
        std::string_view{"AT+COPS?"},                    // query operator
        std::string_view{"AT+CGDCONT=?"},                // test PDP context range
        std::string_view{"ATS0=3"},                      // auto-answer after 3 rings
        std::string_view{"ATS7=60"},                     // 60 s carrier-wait timer
        std::string_view{"ATS0?"},                       // read S0
        std::string_view{"AT&F"},                        // restore factory defaults
    };

    for (auto sv : commands) {
        at::parser p{sv};
        auto result = p.parse_command();
        if (result) {
            std::cout << std::format("  OK  \"{}\"\n", sv);
            print_command(*result);
        } else {
            std::cout << std::format("  ERR \"{}\" — {}\n",
                sv, result.error().message());
        }
    }
}

// ---------------------------------------------------------------------------
// Part 2 — at::stream_parser: handle a continuous serial byte stream
//
// IMPORTANT — STRING_VIEW LIFETIME
//   stream_parser holds an internal linear_buffer.  After each recognised
//   command the parser erases those bytes and shifts the remaining data,
//   which invalidates the string_view fields (raw, prefix, name) that were
//   pointing into the buffer.  The safe pattern is:
//     1. Call feed() with at most one \r-terminated command per call.
//     2. Immediately copy any string_view fields out of the returned
//        at::command objects (see extract() below) before the next feed().
//   This example follows that pattern and also demonstrates fragmented input
//   reassembly for a single command.
// ---------------------------------------------------------------------------

struct CommandInfo {
    std::string        name;
    at::command_type   type;
    std::optional<uint8_t> s_index;
};

static CommandInfo extract(const at::command& cmd) {
    return {std::string{cmd.name}, cmd.type, cmd.s_index};
}

static std::string escape_cr(std::string_view sv) {
    std::string out;
    for (char c : sv) out += (c == '\r') ? "\\r" : std::string(1, c);
    return out;
}

static void process_result(
    std::string_view label,
    const at::stream_parser::result_type& result)
{
    std::cout << std::format("  {:32s} → ", std::format("\"{}\"", escape_cr(label)));
    if (!result.has_value()) {
        std::cout << std::format("error: {}\n", result.error().message());
        return;
    }
    if (result->empty()) {
        std::cout << "(buffering…)\n";
        return;
    }
    for (const auto& cmd : *result) {
        auto info = extract(cmd);
        std::cout << std::format("name={:<10} type={}\n",
            info.name, command_type_name(info.type));
    }
}

static void demo_stream_parser() {
    std::cout << "\n── Stream parser (at::stream_parser) ───────────────────────\n";

    at::stream_parser sp;

    // Feed one \r-terminated command per call — safe pattern for string_view
    // fields.  Fragmented input (bursts 3 + 4) shows reassembly of a single
    // command split across two UART read buffers.
    const std::array<std::string_view, 6> bursts{
        "ATZ\r",            // burst 1 — complete: soft reset
        "ATA\r",            // burst 2 — complete: answer call
        "AT+CM",            // burst 3 — fragment (no \r yet)
        "GF=1\r",           // burst 4 — completes AT+CMGF=1
        "AT+COPS?\r",       // burst 5 — complete: read operator
        "ATS0=3\r",         // burst 6 — complete: set S0 register
    };

    for (auto burst : bursts) {
        auto result = sp.feed(burst);
        process_result(burst, result);
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
    std::cout << std::format(
        "libat v{} (git: {})\n"
        "C++23 zero-copy AT command parser — ITU-T V.250 / 3GPP TS 27.007\n",
        at::version::string,
        at::version::git_hash);

    demo_direct_parser();
    demo_stream_parser();

    std::cout << "\nDone.\n";
}
