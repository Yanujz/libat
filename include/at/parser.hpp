// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signalroute

/**
 * @file at_parser.hpp
 * @brief High-performance C++23 AT command parser library - Production Ready
 * @version 1.2.0
 *
 * Platform-agnostic, header-only AT command parser using C++23 features.
 * Compliant with 3GPP TS 27.007, ITU-T V.250 standards.
 *
 * Changes in v1.2.0:
 * - Added proprietary modem prefix support (^, $, *) for real-world hardware
 *   compatibility (Huawei, u-blox, Motorola, Quectel, Ericsson, Cinterion)
 *
 * Changes in v1.1.0:
 * - Fixed critical ring buffer wrap-around bug (replaced with linear_buffer)
 * - Fixed accidental heap allocations in float parsing and registry lookup
 * - Added missing includes (<generator>, <unordered_map>)
 * - Fixed S3 register compliance (\r termination)
 * - Added heterogeneous lookup for zero-copy command dispatch
 */

#ifndef AT_PARSER_HPP
#define AT_PARSER_HPP

#include <algorithm>
#include <array>
#include <charconv>
#include <concepts>
#include <coroutine>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <expected>
#include <format>
#include <functional>
// std::generator (P2502R2) is available in GCC ≥ 14 (libstdc++) and
// MSVC ≥ 19.35.  GCC 14's <generator> header uses GCC-specific coroutine
// intrinsics; __has_include returns true on Clang when GCC 14 headers are
// installed but the header cannot actually be compiled with Clang.
// Skip the include entirely on any Clang build (Apple or vanilla) and rely
// on the LIBAT_HAS_GENERATOR macro to conditionally compile the stream() API.
#if !defined(__clang__)
#  if __has_include(<generator>)
#    include <generator>
#    if defined(__cpp_lib_generator) && __cpp_lib_generator >= 202207L
#      define LIBAT_HAS_GENERATOR 1
#    else
#      define LIBAT_HAS_GENERATOR 0
#    endif
#  else
#    define LIBAT_HAS_GENERATOR 0
#  endif
#else
#  define LIBAT_HAS_GENERATOR 0
#endif
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace at {

// ============================================================================
// Error Handling
// ============================================================================

enum class parse_error {
    ok = 0,
    incomplete,
    invalid_syntax,
    unknown_command,
    invalid_parameters,
    buffer_overflow,
    checksum_mismatch,
    unexpected_token,
    numeric_conversion_failed,
    string_not_terminated,
};

class parse_error_category : public std::error_category {
public:
    [[nodiscard]] static const parse_error_category& instance() noexcept {
        static parse_error_category cat;
        return cat;
    }

    [[nodiscard]] const char* name() const noexcept override {
        return "at_parse_error";
    }

    [[nodiscard]] std::string message(int ev) const override {
        switch (static_cast<parse_error>(ev)) {
            case parse_error::ok: return "success";
            case parse_error::incomplete: return "incomplete command";
            case parse_error::invalid_syntax: return "invalid syntax";
            case parse_error::unknown_command: return "unknown command";
            case parse_error::invalid_parameters: return "invalid parameters";
            case parse_error::buffer_overflow: return "buffer overflow";
            case parse_error::checksum_mismatch: return "checksum mismatch";
            case parse_error::unexpected_token: return "unexpected token";
            case parse_error::numeric_conversion_failed: return "numeric conversion failed";
            case parse_error::string_not_terminated: return "string not terminated";
            default: return "unknown error";
        }
    }
};

[[nodiscard]] inline std::error_code make_error_code(parse_error e) noexcept {
    return {static_cast<int>(e), parse_error_category::instance()};
}

} // namespace at

namespace std {
    template<> struct is_error_code_enum<at::parse_error> : true_type {};
} // namespace std

namespace at {

// ============================================================================
// Concepts
// ============================================================================

template<typename T>
concept ByteRange = std::ranges::contiguous_range<T> && 
    std::is_same_v<std::ranges::range_value_t<T>, std::byte>;

template<typename T>
concept CharRange = std::ranges::contiguous_range<T> && 
    (std::is_same_v<std::ranges::range_value_t<T>, char> ||
     std::is_same_v<std::ranges::range_value_t<T>, unsigned char>);

template<typename F>
concept CommandHandler = std::invocable<F, std::span<const std::string_view>>;

// ============================================================================
// String Utilities (constexpr where possible)
// ============================================================================

[[nodiscard]] constexpr bool is_at_prefix(std::string_view sv) noexcept {
    return sv.size() >= 2 && sv[0] == 'A' && sv[1] == 'T';
}

[[nodiscard]] constexpr std::string_view trim_left(std::string_view sv) noexcept {
    auto it = std::ranges::find_if_not(sv, [](char c) { 
        return c == ' ' || c == '\t' || c == '\r' || c == '\n'; 
    });
    return {it, sv.end()};
}

[[nodiscard]] constexpr std::string_view trim_right(std::string_view sv) noexcept {
    auto it = std::ranges::find_if_not(sv | std::views::reverse, [](char c) { 
        return c == ' ' || c == '\t' || c == '\r' || c == '\n'; 
    }).base();
    return {sv.begin(), it};
}

[[nodiscard]] constexpr std::string_view trim(std::string_view sv) noexcept {
    return trim_right(trim_left(sv));
}

[[nodiscard]] constexpr bool is_valid_cmd_char(char c) noexcept {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || 
           (c >= '0' && c <= '9') || c == '+' || c == '#' || c == '%' || c == '&' || c == '_';
}

// ============================================================================
// Parameter Types
// ============================================================================

using param_value = std::variant<
    std::monostate,      // empty/optional
    int64_t,             // integer
    double,              // float
    std::string_view,    // string (non-owning view)
    bool                 // boolean flag
>;

class parameters {
public:
    using storage_type = std::vector<std::pair<std::string_view, param_value>>;
    
    parameters() = default;
    
    explicit parameters(std::span<const std::string_view> args) {
        parse_args(args);
    }

    [[nodiscard]] bool empty() const noexcept { return params_.empty(); }
    [[nodiscard]] std::size_t size() const noexcept { return params_.size(); }

    [[nodiscard]] std::optional<param_value> get(std::size_t index) const {
        if (index < params_.size()) {
            return params_[index].second;
        }
        return std::nullopt;
    }

    [[nodiscard]] std::optional<param_value> get(std::string_view name) const {
        for (const auto& [key, val] : params_) {
            if (key == name) return val;
        }
        return std::nullopt;
    }

    template<typename T>
    [[nodiscard]] std::optional<T> get_as(std::size_t index) const {
        auto val = get(index);
        if (!val) return std::nullopt;
        return convert<T>(*val);
    }

    template<typename T>
    [[nodiscard]] std::optional<T> get_as(std::string_view name) const {
        auto val = get(name);
        if (!val) return std::nullopt;
        return convert<T>(*val);
    }

    [[nodiscard]] auto begin() const noexcept { return params_.begin(); }
    [[nodiscard]] auto end() const noexcept { return params_.end(); }

private:
    storage_type params_;

    void parse_args(std::span<const std::string_view> args) {
        params_.reserve(args.size());
        for (const auto& arg : args) {
            if (auto eq_pos = arg.find('='); eq_pos != std::string_view::npos) {
                auto key = trim(arg.substr(0, eq_pos));
                auto val = parse_value(trim(arg.substr(eq_pos + 1)));
                params_.emplace_back(key, val);
            } else {
                params_.emplace_back("", parse_value(arg));
            }
        }
    }

    [[nodiscard]] static param_value parse_value(std::string_view sv) {
        if (sv.empty()) return std::monostate{};
        
        // Check for quoted string
        if (sv.size() >= 2 && sv.front() == '"' && sv.back() == '"') {
            return sv.substr(1, sv.size() - 2);
        }
        
        // Check for boolean
        if (sv == "true" || sv == "TRUE" || sv == "1") return true;
        if (sv == "false" || sv == "FALSE" || sv == "0") return false;
        
        // Try integer
        int64_t int_val{};
        auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), int_val);
        if (ec == std::errc{} && ptr == sv.data() + sv.size()) {
            return int_val;
        }
        
        // Try float using from_chars (zero-allocation on libstdc++ and MSVC).
        // libc++ added from_chars for floating-point types in LLVM 20; fall back
        // to strtod with a small stack buffer on older libc++ to avoid a link
        // error while still keeping stack-only allocation.
        double dbl_val{};
#if defined(_LIBCPP_VERSION) && _LIBCPP_VERSION < 200000
        {
            char buf[64];
            const auto len = std::min(sv.size(), std::size_t{63});
            std::memcpy(buf, sv.data(), len);
            buf[len] = '\0';
            char* end = nullptr;
            dbl_val = std::strtod(buf, &end);
            const auto fptr = sv.data() + (end - buf);
            if (end != buf && fptr == sv.data() + sv.size()) {
                return dbl_val;
            }
        }
#else
        auto [fptr, fec] = std::from_chars(sv.data(), sv.data() + sv.size(), dbl_val);
        if (fec == std::errc{} && fptr == sv.data() + sv.size()) {
            return dbl_val;
        }
#endif
        
        return sv;
    }

    template<typename T>
    [[nodiscard]] static std::optional<T> convert(const param_value& val) {
        return std::visit([]<typename U>(U&& arg) -> std::optional<T> {
            using ArgType = std::decay_t<U>;
            
            if constexpr (std::is_same_v<T, ArgType>) {
                return arg;
            } else if constexpr (std::is_same_v<T, std::string_view> && 
                               std::is_same_v<ArgType, std::string_view>) {
                return arg;
            } else if constexpr (std::is_arithmetic_v<T> && std::is_arithmetic_v<ArgType>) {
                return static_cast<T>(arg);
            } else if constexpr (std::is_same_v<T, bool> && std::is_same_v<ArgType, int64_t>) {
                return arg != 0;
            }
            return std::nullopt;
        }, val);
    }
};

// ============================================================================
// Command Structure
// ============================================================================

enum class command_type : uint8_t {
    execute,        // AT+CMD or AT+CMD=<params>
    read,           // AT+CMD?
    test,           // AT+CMD=?
    set,            // AT+CMD=<value>
    basic,          // ATD, ATA, ATH, etc.
    extended,       // AT+CMD, AT%CMD, AT#CMD, AT&CMD
    s_parameter,    // ATS<n>=<val>, ATS<n>?
    unknown
};

struct command {
    std::string_view raw;
    std::string_view prefix;        // "AT", "at", etc.
    std::string_view name;          // Command name without prefix
    command_type type{command_type::unknown};
    parameters params;
    std::optional<uint8_t> s_index; // For S-parameters
    
    [[nodiscard]] bool is_query() const noexcept {
        return type == command_type::read || type == command_type::test;
    }
    
    [[nodiscard]] bool is_basic() const noexcept {
        return type == command_type::basic;
    }
};

// ============================================================================
// Response Types
// ============================================================================

enum class response_type : uint8_t {
    ok,
    error,
    connect,
    ring,
    no_carrier,
    busy,
    no_answer,
    prompt,         // > for SMS input
    custom,         // +CMD: <data>
    intermediate,   // Data response
    final           // Final result code
};

// ============================================================================
// CME / CMS Error Code Enumerations  (3GPP TS 27.007 §9.2 / TS 27.005 §3.2.5)
// ============================================================================

/// Distinguishes between a plain ERROR, a +CME ERROR, and a +CMS ERROR.
enum class error_kind : uint8_t {
    none,   ///< Not an error response (or plain "ERROR" without a code)
    cme,    ///< +CME ERROR: <code>  (general mobile equipment error)
    cms,    ///< +CMS ERROR: <code>  (SMS-specific error)
};

/// +CME ERROR codes — 3GPP TS 27.007 clause 9.2
enum class cme_error : int {
    phone_failure                  = 0,
    no_connection                  = 1,
    link_reserved                  = 2,
    operation_not_allowed          = 3,
    operation_not_supported        = 4,
    ph_sim_pin_required            = 5,
    ph_fsim_pin_required           = 6,
    ph_fsim_puk_required           = 7,
    sim_not_inserted               = 10,
    sim_pin_required               = 11,
    sim_puk_required               = 12,
    sim_failure                    = 13,
    sim_busy                       = 14,
    sim_wrong                      = 15,
    incorrect_password             = 16,
    sim_pin2_required              = 17,
    sim_puk2_required              = 18,
    memory_full                    = 20,
    invalid_index                  = 21,
    not_found                      = 22,
    memory_failure                 = 23,
    text_string_too_long           = 24,
    invalid_chars_in_text          = 25,
    dial_string_too_long           = 26,
    invalid_chars_in_dial_string   = 27,
    no_network_service             = 30,
    network_timeout                = 31,
    emergency_calls_only           = 32,
    net_pin_required               = 40,
    net_puk_required               = 41,
    net_subset_pin_required        = 42,
    net_subset_puk_required        = 43,
    sp_pin_required                = 44,
    sp_puk_required                = 45,
    corporate_pin_required         = 46,
    corporate_puk_required         = 47,
    unknown                        = 100,
};

/// +CMS ERROR codes — 3GPP TS 27.005 clause 3.2.5
enum class cms_error : int {
    // TP-layer errors (from GSM 04.11 Annex E)
    unassigned_number              = 1,
    operator_barring               = 8,
    call_barred                    = 10,
    transfer_rejected              = 21,
    destination_out_of_service     = 27,
    unidentified_subscriber        = 28,
    facility_rejected              = 29,
    unknown_subscriber             = 30,
    network_out_of_order           = 38,
    temporary_failure              = 41,
    congestion                     = 42,
    resources_unavailable          = 47,
    facility_not_subscribed        = 50,
    facility_not_implemented       = 69,
    invalid_sm_reference           = 81,
    invalid_message                = 95,
    invalid_mandatory_info         = 96,
    message_type_not_implemented   = 97,
    message_incompatible           = 98,
    info_element_not_implemented   = 99,
    protocol_error                 = 111,
    interworking                   = 127,
    // ME (Mobile Equipment) errors
    me_failure                     = 300,
    sms_service_reserved           = 301,
    operation_not_allowed          = 302,
    operation_not_supported        = 303,
    invalid_pdu_mode               = 304,
    invalid_text_mode              = 305,
    sim_not_inserted               = 310,
    sim_pin_required               = 311,
    ph_sim_pin_required            = 312,
    sim_failure                    = 313,
    sim_busy                       = 314,
    sim_wrong                      = 315,
    sim_puk_required               = 316,
    sim_pin2_required              = 317,
    sim_puk2_required              = 318,
    memory_failure                 = 320,
    invalid_memory_index           = 321,
    memory_full                    = 322,
    smsc_address_unknown           = 330,
    no_network_service             = 331,
    network_timeout                = 332,
    no_cnma_ack_expected           = 340,
    unknown                        = 500,
};

/// Returns a human-readable description for a CME error code.
[[nodiscard]] inline std::string_view cme_error_message(cme_error e) noexcept {
    switch (e) {
        case cme_error::phone_failure:               return "phone failure";
        case cme_error::no_connection:               return "no connection to phone";
        case cme_error::link_reserved:               return "phone-adaptor link reserved";
        case cme_error::operation_not_allowed:       return "operation not allowed";
        case cme_error::operation_not_supported:     return "operation not supported";
        case cme_error::ph_sim_pin_required:         return "PH-SIM PIN required";
        case cme_error::ph_fsim_pin_required:        return "PH-FSIM PIN required";
        case cme_error::ph_fsim_puk_required:        return "PH-FSIM PUK required";
        case cme_error::sim_not_inserted:            return "SIM not inserted";
        case cme_error::sim_pin_required:            return "SIM PIN required";
        case cme_error::sim_puk_required:            return "SIM PUK required";
        case cme_error::sim_failure:                 return "SIM failure";
        case cme_error::sim_busy:                    return "SIM busy";
        case cme_error::sim_wrong:                   return "SIM wrong";
        case cme_error::incorrect_password:          return "incorrect password";
        case cme_error::sim_pin2_required:           return "SIM PIN2 required";
        case cme_error::sim_puk2_required:           return "SIM PUK2 required";
        case cme_error::memory_full:                 return "memory full";
        case cme_error::invalid_index:               return "invalid index";
        case cme_error::not_found:                   return "not found";
        case cme_error::memory_failure:              return "memory failure";
        case cme_error::text_string_too_long:        return "text string too long";
        case cme_error::invalid_chars_in_text:       return "invalid characters in text string";
        case cme_error::dial_string_too_long:        return "dial string too long";
        case cme_error::invalid_chars_in_dial_string:return "invalid characters in dial string";
        case cme_error::no_network_service:          return "no network service";
        case cme_error::network_timeout:             return "network timeout";
        case cme_error::emergency_calls_only:        return "network not allowed, emergency calls only";
        case cme_error::net_pin_required:            return "network personalization PIN required";
        case cme_error::net_puk_required:            return "network personalization PUK required";
        case cme_error::net_subset_pin_required:     return "network subset personalization PIN required";
        case cme_error::net_subset_puk_required:     return "network subset personalization PUK required";
        case cme_error::sp_pin_required:             return "service provider personalization PIN required";
        case cme_error::sp_puk_required:             return "service provider personalization PUK required";
        case cme_error::corporate_pin_required:      return "corporate personalization PIN required";
        case cme_error::corporate_puk_required:      return "corporate personalization PUK required";
        case cme_error::unknown:                     return "unknown error";
        default:                                     return "unknown CME error";
    }
}

/// Returns a human-readable description for a CMS error code.
[[nodiscard]] inline std::string_view cms_error_message(cms_error e) noexcept {
    switch (e) {
        case cms_error::unassigned_number:           return "unassigned (unallocated) number";
        case cms_error::operator_barring:            return "operator determined barring";
        case cms_error::call_barred:                 return "call barred";
        case cms_error::transfer_rejected:           return "short message transfer rejected";
        case cms_error::destination_out_of_service:  return "destination out of service";
        case cms_error::unidentified_subscriber:     return "unidentified subscriber";
        case cms_error::facility_rejected:           return "facility rejected";
        case cms_error::unknown_subscriber:          return "unknown subscriber";
        case cms_error::network_out_of_order:        return "network out of order";
        case cms_error::temporary_failure:           return "temporary failure";
        case cms_error::congestion:                  return "congestion";
        case cms_error::resources_unavailable:       return "resources unavailable";
        case cms_error::facility_not_subscribed:     return "requested facility not subscribed";
        case cms_error::facility_not_implemented:    return "requested facility not implemented";
        case cms_error::invalid_sm_reference:        return "invalid short message transfer reference";
        case cms_error::invalid_message:             return "invalid message";
        case cms_error::invalid_mandatory_info:      return "invalid mandatory information";
        case cms_error::message_type_not_implemented:return "message type not implemented";
        case cms_error::message_incompatible:        return "message not compatible with protocol state";
        case cms_error::info_element_not_implemented:return "information element not implemented";
        case cms_error::protocol_error:              return "protocol error";
        case cms_error::interworking:                return "interworking";
        case cms_error::me_failure:                  return "ME failure";
        case cms_error::sms_service_reserved:        return "SMS service of ME reserved";
        case cms_error::operation_not_allowed:       return "operation not allowed";
        case cms_error::operation_not_supported:     return "operation not supported";
        case cms_error::invalid_pdu_mode:            return "invalid PDU mode parameter";
        case cms_error::invalid_text_mode:           return "invalid text mode parameter";
        case cms_error::sim_not_inserted:            return "(U)SIM not inserted";
        case cms_error::sim_pin_required:            return "(U)SIM PIN required";
        case cms_error::ph_sim_pin_required:         return "PH-(U)SIM PIN required";
        case cms_error::sim_failure:                 return "(U)SIM failure";
        case cms_error::sim_busy:                    return "(U)SIM busy";
        case cms_error::sim_wrong:                   return "(U)SIM wrong";
        case cms_error::sim_puk_required:            return "(U)SIM PUK required";
        case cms_error::sim_pin2_required:           return "(U)SIM PIN2 required";
        case cms_error::sim_puk2_required:           return "(U)SIM PUK2 required";
        case cms_error::memory_failure:              return "memory failure";
        case cms_error::invalid_memory_index:        return "invalid memory index";
        case cms_error::memory_full:                 return "memory full";
        case cms_error::smsc_address_unknown:        return "SMSC address unknown";
        case cms_error::no_network_service:          return "no network service";
        case cms_error::network_timeout:             return "network timeout";
        case cms_error::no_cnma_ack_expected:        return "no +CNMA acknowledgement expected";
        case cms_error::unknown:                     return "unknown error";
        default:                                     return "unknown CMS error";
    }
}

struct response {
    response_type type{response_type::ok};
    error_kind    error_source{error_kind::none};
    std::string_view raw;
    std::optional<command> associated_cmd;
    std::vector<std::string_view> data_lines;
    int error_code{0};  // Numeric code from +CME ERROR / +CMS ERROR

    /// Returns the error as a cme_error value when error_source == error_kind::cme.
    [[nodiscard]] std::optional<cme_error> as_cme_error() const noexcept {
        if (error_source != error_kind::cme) return std::nullopt;
        return static_cast<cme_error>(error_code);
    }

    /// Returns the error as a cms_error value when error_source == error_kind::cms.
    [[nodiscard]] std::optional<cms_error> as_cms_error() const noexcept {
        if (error_source != error_kind::cms) return std::nullopt;
        return static_cast<cms_error>(error_code);
    }
};

// ============================================================================
// Low-level Tokenizer (Coroutine-based for streaming)
// ============================================================================

struct token {
    enum class type : uint8_t {
        at_prefix,
        plus,
        hash,
        percent,
        ampersand,
        caret,      ///< '^'  — Huawei / Cinterion (AT^SYSCFG, AT^SYSINFO …)
        dollar,     ///< '$'  — u-blox / Telit      (AT$GPSP, AT$GPSACP …)
        asterisk,   ///< '*'  — Motorola / Quectel   (AT*CNTI, AT*E2SMSPR …)
        identifier,
        number,
        string,
        equal,
        question,
        comma,
        semicolon,
        colon,
        cr,
        lf,
        crlf,
        eof,
        error
    };
    
    type tok_type{type::error};
    std::string_view lexeme;
    std::size_t position{0};
};

class tokenizer {
public:
    explicit tokenizer(std::string_view input) : input_(input), pos_(0) {}
    
    [[nodiscard]] token next() {
        skip_whitespace();
        
        if (pos_ >= input_.size()) {
            return make_token(token::type::eof, 0);
        }
        
        const char c = input_[pos_];
        const auto start_pos = pos_;
        
        switch (c) {
            case 'A':
            case 'a':
                if (peek_match("AT") || peek_match("at")) {
                    advance(2);
                    return make_token(token::type::at_prefix, start_pos, 2);
                }
                return read_identifier();
            case '+': advance(); return make_token(token::type::plus, start_pos);
            case '#': advance(); return make_token(token::type::hash, start_pos);
            case '%': advance(); return make_token(token::type::percent, start_pos);
            case '&': advance(); return make_token(token::type::ampersand, start_pos);
            case '^': advance(); return make_token(token::type::caret,     start_pos);
            case '$': advance(); return make_token(token::type::dollar,    start_pos);
            case '*': advance(); return make_token(token::type::asterisk,  start_pos);
            case '=': advance(); return make_token(token::type::equal, start_pos);
            case '?': advance(); return make_token(token::type::question, start_pos);
            case ',': advance(); return make_token(token::type::comma, start_pos);
            case ';': advance(); return make_token(token::type::semicolon, start_pos);
            case ':': advance(); return make_token(token::type::colon, start_pos);
            case '"': return read_string();
            case '\r':
                advance();
                if (match('\n')) {
                    return make_token(token::type::crlf, start_pos, 2);
                }
                return make_token(token::type::cr, start_pos);
            case '\n':
                advance();
                return make_token(token::type::lf, start_pos);
            default:
                if (std::isdigit(static_cast<unsigned char>(c))) {
                    return read_number();
                }
                if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
                    return read_identifier();
                }
                advance();
                return make_token(token::type::error, start_pos);
        }
    }
    
    // C++23 generator for streaming tokens.
    // Only available when std::generator is provided by the standard library
    // (GCC ≥ 14 / MSVC ≥ 19.38).  Use tokenizer::next() directly on Clang.
#if LIBAT_HAS_GENERATOR
    [[nodiscard]] static std::generator<token> stream(std::string_view input) {
        tokenizer t(input);
        while (true) {
            auto tok = t.next();
            co_yield tok;
            if (tok.tok_type == token::type::eof || tok.tok_type == token::type::error) {
                break;
            }
        }
    }
#endif // LIBAT_HAS_GENERATOR

private:
    std::string_view input_;
    std::size_t pos_;
    
    [[nodiscard]] char peek(std::size_t offset = 0) const noexcept {
        if (pos_ + offset < input_.size()) {
            return input_[pos_ + offset];
        }
        return '\0';
    }
    
    [[nodiscard]] bool peek_match(std::string_view s) const noexcept {
        return input_.substr(pos_, s.size()) == s;
    }
    
    void advance(std::size_t n = 1) noexcept {
        pos_ = std::min(pos_ + n, input_.size());
    }
    
    [[nodiscard]] bool match(char expected) noexcept {
        if (peek() == expected) {
            advance();
            return true;
        }
        return false;
    }
    
    void skip_whitespace() noexcept {
        while (pos_ < input_.size() && 
               (input_[pos_] == ' ' || input_[pos_] == '\t')) {
            ++pos_;
        }
    }
    
    [[nodiscard]] token make_token(token::type t, std::size_t start, 
                                    std::size_t len = 1) const {
        return {t, input_.substr(start, len), start};
    }
    
    [[nodiscard]] token read_identifier() {
        const auto start = pos_;
        while (pos_ < input_.size() && is_valid_cmd_char(input_[pos_])) {
            ++pos_;
        }
        return make_token(token::type::identifier, start, pos_ - start);
    }
    
    [[nodiscard]] token read_number() {
        const auto start = pos_;
        while (pos_ < input_.size() && std::isdigit(static_cast<unsigned char>(input_[pos_]))) {
            ++pos_;
        }
        if (pos_ < input_.size() && input_[pos_] == '.') {
            ++pos_;
            while (pos_ < input_.size() && std::isdigit(static_cast<unsigned char>(input_[pos_]))) {
                ++pos_;
            }
        }
        return make_token(token::type::number, start, pos_ - start);
    }
    
    [[nodiscard]] token read_string() {
        const auto start = pos_;
        advance(); // opening quote
        while (pos_ < input_.size() && input_[pos_] != '"') {
            if (input_[pos_] == '\\' && pos_ + 1 < input_.size()) {
                advance(2);
            } else {
                advance();
            }
        }
        if (pos_ >= input_.size()) {
            return {token::type::error, input_.substr(start), start};
        }
        advance(); // closing quote
        return make_token(token::type::string, start, pos_ - start);
    }
};

// ============================================================================
// Parser
// ============================================================================

class parser {
public:
    explicit parser(std::string_view input) 
        : input_(input), tokenizer_(input), current_(tokenizer_.next()) {}
    
    [[nodiscard]] std::expected<command, std::error_code> parse_command() {
        cmd_ = command{};
        cmd_.raw = input_;
        
        // Expect AT prefix
        if (current_.tok_type != token::type::at_prefix) {
            return std::unexpected(make_error_code(parse_error::invalid_syntax));
        }
        cmd_.prefix = current_.lexeme;
        advance();
        
        // Parse command body
        if (auto err = parse_command_body(); err != parse_error::ok) {
            return std::unexpected(make_error_code(err));
        }
        
        return cmd_;
    }
    
    [[nodiscard]] std::expected<response, std::error_code> parse_response() {
        response resp;
        resp.raw = input_;
        
        // Check for final result codes
        auto trimmed = trim(input_);
        
        if (trimmed == "OK" || trimmed == "ok") {
            resp.type = response_type::ok;
            return resp;
        }
        if (trimmed == "ERROR" || trimmed == "error") {
            resp.type = response_type::error;
            return resp;
        }
        if (trimmed.starts_with("CONNECT")) {
            resp.type = response_type::connect;
            return resp;
        }
        if (trimmed == "RING") {
            resp.type = response_type::ring;
            return resp;
        }
        if (trimmed == "NO CARRIER") {
            resp.type = response_type::no_carrier;
            return resp;
        }
        if (trimmed == "BUSY") {
            resp.type = response_type::busy;
            return resp;
        }
        if (trimmed == "NO ANSWER") {
            resp.type = response_type::no_answer;
            return resp;
        }
        if (trimmed == "> ") {
            resp.type = response_type::prompt;
            return resp;
        }
        
        // Check for extended error responses
        if (trimmed.starts_with("+CME ERROR:") || trimmed.starts_with("+CMS ERROR:")) {
            resp.type = response_type::error;
            // Parse error code
            auto colon_pos = trimmed.find(':');
            if (colon_pos != std::string_view::npos) {
                auto code_str = trim(trimmed.substr(colon_pos + 1));
                std::from_chars(code_str.data(), code_str.data() + code_str.size(), resp.error_code);
            }
            return resp;
        }
        
        // Check for information responses (+CMD: data)
        if (trimmed.starts_with('+')) {
            resp.type = response_type::custom;
            // Split lines
            resp.data_lines = split_lines(trimmed);
            return resp;
        }
        
        resp.type = response_type::intermediate;
        resp.data_lines = split_lines(trimmed);
        return resp;
    }

private:
    std::string_view input_;
    tokenizer tokenizer_;
    token current_;
    command cmd_;
    
    void advance() {
        current_ = tokenizer_.next();
    }
    
    [[nodiscard]] parse_error parse_command_body() {
        // Check for S-parameter (ATS<n>)
        if (current_.tok_type == token::type::identifier && 
            (current_.lexeme.starts_with('S') || current_.lexeme.starts_with('s'))) {
            return parse_s_parameter();
        }
        
        // Check for basic commands (D, A, H, O, E, I, L, M, N, P, Q, T, V, X, Z, etc.)
        if (current_.tok_type == token::type::identifier && 
            current_.lexeme.size() == 1) {
            return parse_basic_command();
        }
        
        // Extended command (+, #, %, &)
        return parse_extended_command();
    }
    
    [[nodiscard]] parse_error parse_s_parameter() {
        cmd_.type = command_type::s_parameter;
        auto& lex = current_.lexeme;
        
        // Parse S<n>
        if (lex.size() < 2 || (lex[0] != 'S' && lex[0] != 's')) {
            return parse_error::invalid_syntax;
        }
        
        uint8_t idx{};
        auto [ptr, ec] = std::from_chars(lex.data() + 1, lex.data() + lex.size(), idx);
        if (ec != std::errc{}) {
            return parse_error::invalid_syntax;
        }
        cmd_.s_index = idx;
        cmd_.name = lex;
        advance();
        
        // Check for =<val> or ?
        if (current_.tok_type == token::type::equal) {
            advance();
            if (current_.tok_type == token::type::number) {
                std::vector<std::string_view> args{current_.lexeme};
                cmd_.params = parameters(std::span(args));
                advance();
            }
        } else if (current_.tok_type == token::type::question) {
            cmd_.type = command_type::read;
            advance();
        }
        
        return parse_error::ok;
    }
    
    [[nodiscard]] parse_error parse_basic_command() {
        cmd_.type = command_type::basic;
        cmd_.name = current_.lexeme;
        advance();
        
        // Basic commands may have dial string or parameters
        std::vector<std::string_view> args;
        while (current_.tok_type != token::type::eof && 
               current_.tok_type != token::type::crlf &&
               current_.tok_type != token::type::cr &&
               current_.tok_type != token::type::lf &&
               current_.tok_type != token::type::semicolon) {
            args.push_back(current_.lexeme);
            advance();
        }
        
        if (!args.empty()) {
            cmd_.params = parameters(std::span(args));
        }
        
        return parse_error::ok;
    }
    
    [[nodiscard]] parse_error parse_extended_command() {
        // Parse prefix (+, #, %, &)
        char prefix = '\0';
        switch (current_.tok_type) {
            case token::type::plus:      prefix = '+'; break;
            case token::type::hash:      prefix = '#'; break;
            case token::type::percent:   prefix = '%'; break;
            case token::type::ampersand: prefix = '&'; break;
            case token::type::caret:     prefix = '^'; break;  // Huawei / Cinterion
            case token::type::dollar:    prefix = '$'; break;  // u-blox / Telit
            case token::type::asterisk:  prefix = '*'; break;  // Motorola / Quectel
            default: return parse_error::invalid_syntax;
        }
        advance();
        
        // Expect identifier
        if (current_.tok_type != token::type::identifier) {
            return parse_error::invalid_syntax;
        }
        
        cmd_.name = current_.lexeme;
        cmd_.type = command_type::extended;
        advance();
        
        // Check for action
        if (current_.tok_type == token::type::question) {
            cmd_.type = command_type::read;
            advance();
        } else if (current_.tok_type == token::type::equal) {
            advance();
            if (current_.tok_type == token::type::question) {
                cmd_.type = command_type::test;
                advance();
            } else {
                cmd_.type = command_type::set;
                auto err = parse_parameters();
                if (err != parse_error::ok) return err;
            }
        }
        
        return parse_error::ok;
    }
    
    [[nodiscard]] parse_error parse_parameters() {
        std::vector<std::string_view> args;
        
        while (current_.tok_type != token::type::eof && 
               current_.tok_type != token::type::crlf &&
               current_.tok_type != token::type::cr &&
               current_.tok_type != token::type::lf &&
               current_.tok_type != token::type::semicolon) {
            
            if (current_.tok_type == token::type::comma) {
                // Empty parameter
                args.emplace_back();
            } else if (current_.tok_type == token::type::string) {
                args.push_back(current_.lexeme);
            } else if (current_.tok_type == token::type::number || 
                       current_.tok_type == token::type::identifier) {
                args.push_back(current_.lexeme);
            } else {
                break;
            }
            
            advance();
            
            if (current_.tok_type == token::type::comma) {
                advance();
            } else {
                break;
            }
        }
        
        cmd_.params = parameters(std::span(args));
        return parse_error::ok;
    }
    
    [[nodiscard]] static std::vector<std::string_view> split_lines(std::string_view sv) {
        std::vector<std::string_view> lines;
        std::size_t start = 0;
        while (start < sv.size()) {
            auto end = sv.find_first_of("\r\n", start);
            if (end == std::string_view::npos) {
                lines.push_back(trim(sv.substr(start)));
                break;
            }
            if (end > start) {
                lines.push_back(trim(sv.substr(start, end - start)));
            }
            start = end + 1;
            if (start < sv.size() && sv[start - 1] == '\r' && sv[start] == '\n') {
                ++start;
            }
        }
        return lines;
    }
};

// ============================================================================
// Builder for constructing AT commands
// ============================================================================

class command_builder {
public:
    [[nodiscard]] static std::string build(std::string_view name, 
                                            command_type type = command_type::execute,
                                            std::span<const std::string> params = {}) {
        std::string result = "AT";
        
        if (type == command_type::basic) {
            result += name;
        } else {
            result += name;
        }
        
        switch (type) {
            case command_type::read:
                result += "?";
                break;
            case command_type::test:
                result += "=?";
                break;
            case command_type::set:
            case command_type::execute:
                if (!params.empty()) {
                    result += "=";
                    for (std::size_t i = 0; i < params.size(); ++i) {
                        if (i > 0) result += ",";
                        // Quote strings with spaces or special chars
                        if (needs_quoting(params[i])) {
                            result += std::format("\"{}\"", params[i]);
                        } else {
                            result += params[i];
                        }
                    }
                }
                break;
            default:
                break;
        }
        
        result += "\r\n";
        return result;
    }
    
    [[nodiscard]] static std::string build_s_parameter(uint8_t index, 
                                                        std::optional<int> value = std::nullopt) {
        if (value) {
            return std::format("ATS{}={}\r\n", index, *value);
        }
        return std::format("ATS{}?\r\n", index);
    }

private:
    [[nodiscard]] static bool needs_quoting(std::string_view sv) {
        return sv.find_first_of(" ,;=\"\r\n") != std::string_view::npos;
    }
};

// ============================================================================
// Linear Buffer (Fixed ring buffer wrap-around bug)
// ============================================================================

class linear_buffer {
public:
    [[nodiscard]] std::size_t size() const noexcept {
        return buffer_.size();
    }
    
    [[nodiscard]] bool empty() const noexcept {
        return buffer_.empty();
    }
    
    [[nodiscard]] std::size_t capacity() const noexcept {
        return buffer_.capacity();
    }
    
    void push(std::string_view data) {
        buffer_.insert(buffer_.end(), data.begin(), data.end());
    }
    
    [[nodiscard]] bool push(char c) {
        buffer_.push_back(c);
        return true;
    }
    
    [[nodiscard]] std::string_view peek_line() const noexcept {
        auto it = std::ranges::find(buffer_, '\r');
        if (it != buffer_.end()) {
            // Include the \r in the line
            return std::string_view(buffer_.data(), std::distance(buffer_.begin(), it) + 1);
        }
        return {};
    }
    
    void consume(std::size_t n) {
        buffer_.erase(buffer_.begin(), buffer_.begin() + std::min(n, buffer_.size()));
    }
    
    void clear() noexcept {
        buffer_.clear();
    }
    
    [[nodiscard]] std::string_view view(std::size_t offset, std::size_t len) const noexcept {
        if (offset >= buffer_.size()) return {};
        len = std::min(len, buffer_.size() - offset);
        return std::string_view(buffer_.data() + offset, len);
    }

private:
    std::vector<char> buffer_;
};

// ============================================================================
// Stream Parser (for handling continuous input)
// ============================================================================

class stream_parser {
public:
    using result_type = std::expected<std::vector<command>, std::error_code>;

    [[nodiscard]] result_type
    feed(std::string_view data) {
        buffer_.push(data);
        return process_buffer();
    }
    
    [[nodiscard]] std::expected<std::optional<command>, std::error_code> 
    feed_char(char c) {
        buffer_.push(c);
        return try_parse_one();
    }
    
    void reset() noexcept {
        buffer_.clear();
    }

private:
    linear_buffer buffer_;
    
    [[nodiscard]] std::expected<std::vector<command>, std::error_code> 
    process_buffer() {
        std::vector<command> commands;
        
        while (true) {
            auto result = try_parse_one();
            if (!result) {
                return std::unexpected(result.error());
            }
            if (!*result) break;
            commands.push_back(**result);
        }
        
        return commands;
    }
    
    [[nodiscard]] std::expected<std::optional<command>, std::error_code> 
    try_parse_one() {
        auto line = buffer_.peek_line();
        if (line.empty()) {
            return std::nullopt; // Need more data
        }
        
        // Check if we have a complete command (ends with \r per S3 register default)
        if (!line.ends_with('\r')) {
            return std::nullopt;
        }
        
        // Remove trailing \r for parsing
        auto to_parse = trim(line.substr(0, line.size() - 1));
        if (to_parse.empty()) {
            buffer_.consume(line.size());
            return std::nullopt;
        }
        
        parser p(to_parse);
        auto result = p.parse_command();
        if (result) {
            buffer_.consume(line.size());
            return *result;
        }
        
        // If parsing failed, check if it's just incomplete
        if (result.error() == make_error_code(parse_error::incomplete)) {
            return std::nullopt;
        }
        
        return std::unexpected(result.error());
    }
};

// ============================================================================
// Command Registry and Dispatcher (Fixed heterogeneous lookup)
// ============================================================================

// Transparent hash for std::string_view lookups without allocation
struct string_hash {
    using is_transparent = void; // Enables heterogeneous lookup in C++20/23
    
    [[nodiscard]] std::size_t operator()(std::string_view sv) const noexcept {
        return std::hash<std::string_view>{}(sv);
    }
};

class command_registry {
public:
    using handler_type = std::function<void(const command&)>;
    
    void register_handler(std::string_view name, handler_type handler) {
        handlers_[std::string{name}] = std::move(handler);
    }
    
    template<typename F>
    void register_handler(std::string_view name, F&& f) {
        handlers_[std::string{name}] = [f = std::forward<F>(f)](const command& cmd) {
            std::invoke(f, cmd);
        };
    }
    
    // Zero-copy lookup using heterogeneous lookup (C++20 feature)
    [[nodiscard]] bool dispatch(const command& cmd) const {
        if (auto it = handlers_.find(cmd.name); it != handlers_.end()) {
            it->second(cmd);
            return true;
        }
        return false;
    }
    
    void clear() noexcept {
        handlers_.clear();
    }

private:
    // Uses string_hash for transparent lookup, std::equal_to<> for heterogeneous comparison
    std::unordered_map<std::string, handler_type, string_hash, std::equal_to<>> handlers_;
};

// ============================================================================
// Utility Functions
// ============================================================================

[[nodiscard]] inline bool validate_checksum(std::string_view data, 
                                           std::string_view expected) noexcept {
    uint8_t checksum = 0;
    for (char c : data) {
        checksum ^= static_cast<uint8_t>(c);
    }
    
    uint8_t exp_val{};
    auto [ptr, ec] = std::from_chars(expected.data(), 
                                      expected.data() + expected.size(), 
                                      exp_val, 16);
    return ec == std::errc{} && checksum == exp_val;
}

[[nodiscard]] inline std::string compute_checksum(std::string_view data) {
    uint8_t checksum = 0;
    for (char c : data) {
        checksum ^= static_cast<uint8_t>(c);
    }
    return std::format("{:02X}", checksum);
}

// ============================================================================
// C++23 Ranges Views for AT command processing
// ============================================================================

namespace views {

[[nodiscard]] inline auto split_params(char delimiter = ',') {
    return std::views::split(delimiter) | 
           std::views::transform([](auto&& r) -> std::string_view {
               auto sv = std::string_view(&*r.begin(), std::ranges::distance(r));
               return trim(sv);
           });
}

[[nodiscard]] inline auto parse_commands() {
    return std::views::transform([](std::string_view line) 
        -> std::expected<command, std::error_code> {
        parser p(line);
        return p.parse_command();
    });
}

} // namespace views

} // namespace at

#endif // AT_PARSER_HPP