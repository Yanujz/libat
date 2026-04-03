/**
 * @file test_parser.cpp
 * @brief Google Test suite for the libat AT command parser.
 *
 * All test strings are 100 % compliant with ITU-T V.250 / 3GPP TS 27.007:
 *   • Basic commands    : single letter after "AT" (ATA, ATH, ATZ, ATI …)
 *   • Extended commands : AT+CMD, AT#CMD, AT%CMD, AT&CMD
 *   • S-parameters      : ATS<n>=<val>, ATS<n>?
 *
 * at::parser::parse_command() returns std::expected<at::command, std::error_code>.
 * at::stream_parser::feed()   returns std::expected<std::vector<at::command>, std::error_code>.
 *
 * Note on string_view lifetime with stream_parser:
 *   The linear_buffer inside stream_parser erases consumed bytes after each
 *   parsed command, which invalidates the string_view fields (raw, prefix,
 *   name) stored in returned at::command objects.  Stream-parser tests
 *   therefore only inspect value-type fields (command_type, s_index, and
 *   numeric / boolean parameter values) that are safe to read after feed().
 */

#include <gtest/gtest.h>
#include <at/parser.hpp>

// ============================================================================
// Helpers
// ============================================================================

/// Convenience wrapper: parse a single AT command string.
static at::parser make_parser(std::string_view sv) {
    return at::parser{sv};
}

// ============================================================================
// Basic commands  (ITU-T V.250 §6 — single-letter mnemonic codes)
// ============================================================================

TEST(ParserBasicCommands, AnswerCall) {
    auto p      = make_parser("ATA");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type,   at::command_type::basic);
    EXPECT_EQ(result->name,   "A");
    EXPECT_TRUE(result->params.empty());
}

TEST(ParserBasicCommands, HangUp) {
    auto p      = make_parser("ATH");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::basic);
    EXPECT_EQ(result->name, "H");
}

TEST(ParserBasicCommands, SoftReset) {
    auto p      = make_parser("ATZ");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::basic);
    EXPECT_EQ(result->name, "Z");
}

TEST(ParserBasicCommands, RequestIdentification) {
    auto p      = make_parser("ATI");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::basic);
    EXPECT_EQ(result->name, "I");
}

TEST(ParserBasicCommands, EchoControl) {
    // ATE — echo command (no trailing digit; digit variant is AT+CMEE style)
    auto p      = make_parser("ATE");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::basic);
    EXPECT_EQ(result->name, "E");
}

TEST(ParserBasicCommands, VerboseMode) {
    auto p      = make_parser("ATV");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::basic);
    EXPECT_EQ(result->name, "V");
}

// ============================================================================
// Extended commands — execute (AT+CMD with no suffix)
// ============================================================================

TEST(ParserExtendedCommands, ExecuteCSQ) {
    // AT+CSQ — signal quality (3GPP TS 27.007 §8.5)
    auto p      = make_parser("AT+CSQ");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::extended);
    EXPECT_EQ(result->name, "CSQ");
}

TEST(ParserExtendedCommands, ExecuteCREG) {
    // AT+CREG — network registration (3GPP TS 27.007 §10.1.19)
    auto p      = make_parser("AT+CREG");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::extended);
    EXPECT_EQ(result->name, "CREG");
}

// ============================================================================
// Extended commands — read (AT+CMD?)
// ============================================================================

TEST(ParserExtendedCommands, ReadCOPS) {
    // AT+COPS? — query operator selection (3GPP TS 27.007 §7.3)
    auto p      = make_parser("AT+COPS?");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::read);
    EXPECT_EQ(result->name, "COPS");
    EXPECT_TRUE(result->is_query());
}

TEST(ParserExtendedCommands, ReadCREG) {
    auto p      = make_parser("AT+CREG?");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::read);
    EXPECT_EQ(result->name, "CREG");
}

TEST(ParserExtendedCommands, ReadCMGF) {
    // AT+CMGF? — SMS message format (3GPP TS 27.005 §3.2.3)
    auto p      = make_parser("AT+CMGF?");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::read);
    EXPECT_EQ(result->name, "CMGF");
}

// ============================================================================
// Extended commands — test (AT+CMD=?)
// ============================================================================

TEST(ParserExtendedCommands, TestCGDCONT) {
    // AT+CGDCONT=? — PDP context definition test (3GPP TS 27.007 §10.1.1)
    auto p      = make_parser("AT+CGDCONT=?");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::test);
    EXPECT_EQ(result->name, "CGDCONT");
    EXPECT_TRUE(result->is_query());
}

TEST(ParserExtendedCommands, TestCOPS) {
    auto p      = make_parser("AT+COPS=?");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::test);
    EXPECT_EQ(result->name, "COPS");
}

// ============================================================================
// Extended commands — set (AT+CMD=<params>)
// ============================================================================

TEST(ParserExtendedCommands, SetCMGFTextMode) {
    // AT+CMGF=1 — select SMS text mode
    auto p      = make_parser("AT+CMGF=1");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::set);
    EXPECT_EQ(result->name, "CMGF");

    auto val = result->params.get_as<int64_t>(0);
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 1);
}

TEST(ParserExtendedCommands, SetCREGMode) {
    // AT+CREG=2 — enable registration URC with location info
    auto p      = make_parser("AT+CREG=2");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::set);
    EXPECT_EQ(result->name, "CREG");

    auto val = result->params.get_as<int64_t>(0);
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 2);
}

TEST(ParserExtendedCommands, SetCGDCONTWithStringParams) {
    // AT+CGDCONT=1,"IP","internet" — define PDP context
    const std::string input{R"(AT+CGDCONT=1,"IP","internet")"};
    auto p      = make_parser(input);
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::set);
    EXPECT_EQ(result->name, "CGDCONT");
    EXPECT_EQ(result->params.size(), 3u);

    // First parameter: PDP context ID (integer)
    auto cid = result->params.get_as<int64_t>(0);
    ASSERT_TRUE(cid.has_value());
    EXPECT_EQ(*cid, 1);

    // Second parameter: PDP type ("IP")
    auto pdp_type = result->params.get_as<std::string_view>(1);
    ASSERT_TRUE(pdp_type.has_value());
    EXPECT_EQ(*pdp_type, "IP");

    // Third parameter: APN
    auto apn = result->params.get_as<std::string_view>(2);
    ASSERT_TRUE(apn.has_value());
    EXPECT_EQ(*apn, "internet");
}

TEST(ParserExtendedCommands, SetCOPSManual) {
    // AT+COPS=1,2,"26201" — manually select operator in numeric format
    const std::string input{R"(AT+COPS=1,2,"26201")"};
    auto p      = make_parser(input);
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::set);
    EXPECT_EQ(result->name, "COPS");
    EXPECT_EQ(result->params.size(), 3u);

    auto mode   = result->params.get_as<int64_t>(0);
    auto format = result->params.get_as<int64_t>(1);
    ASSERT_TRUE(mode.has_value());
    ASSERT_TRUE(format.has_value());
    EXPECT_EQ(*mode,   1);
    EXPECT_EQ(*format, 2);
}

// ============================================================================
// Extended commands — ampersand class (AT&CMD per ITU-T V.250 §5.4.1)
// ============================================================================

TEST(ParserExtendedCommands, AmpersandFactoryDefaults) {
    // AT&F — restore factory settings
    auto p      = make_parser("AT&F");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::extended);
    EXPECT_EQ(result->name, "F");
}

// ============================================================================
// S-parameters  (ITU-T V.250 §6.3 — S-register commands)
// ============================================================================

TEST(ParserSParameters, SetS0AutoAnswer) {
    // ATS0=3 — auto-answer after 3 rings
    auto p      = make_parser("ATS0=3");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::s_parameter);
    ASSERT_TRUE(result->s_index.has_value());
    EXPECT_EQ(*result->s_index, 0u);

    auto val = result->params.get_as<int64_t>(0);
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 3);
}

TEST(ParserSParameters, SetS7ResponseTimer) {
    // ATS7=60 — wait 60 s for carrier (3GPP TS 27.007 S-register table)
    auto p      = make_parser("ATS7=60");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::s_parameter);
    ASSERT_TRUE(result->s_index.has_value());
    EXPECT_EQ(*result->s_index, 7u);

    auto val = result->params.get_as<int64_t>(0);
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 60);
}

TEST(ParserSParameters, ReadS0) {
    // ATS0? — read current value of S0
    auto p      = make_parser("ATS0?");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::read);
    ASSERT_TRUE(result->s_index.has_value());
    EXPECT_EQ(*result->s_index, 0u);
    EXPECT_TRUE(result->is_query());
}

TEST(ParserSParameters, ReadS3LineTerminatorRegister) {
    // ATS3? — read S3 (line termination character register)
    auto p      = make_parser("ATS3?");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::read);
    ASSERT_TRUE(result->s_index.has_value());
    EXPECT_EQ(*result->s_index, 3u);
}

TEST(ParserSParameters, SetS3MaxValue) {
    // ATS3=255 — boundary value for an 8-bit S-register
    auto p      = make_parser("ATS3=255");
    auto result = p.parse_command();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, at::command_type::s_parameter);
    ASSERT_TRUE(result->s_index.has_value());
    EXPECT_EQ(*result->s_index, 3u);

    auto val = result->params.get_as<int64_t>(0);
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 255);
}

// ============================================================================
// Error cases
// ============================================================================

TEST(ParserErrorCases, EmptyInput) {
    auto p      = make_parser("");
    auto result = p.parse_command();
    EXPECT_FALSE(result.has_value());
}

TEST(ParserErrorCases, MissingATPrefix) {
    auto p      = make_parser("+COPS?");
    auto result = p.parse_command();
    EXPECT_FALSE(result.has_value());
}

TEST(ParserErrorCases, RandomGarbage) {
    auto p      = make_parser("HELLO WORLD");
    auto result = p.parse_command();
    EXPECT_FALSE(result.has_value());
}

TEST(ParserErrorCases, JustATPrefix) {
    // "AT" alone — no command body; should not parse as a valid command.
    auto p      = make_parser("AT");
    auto result = p.parse_command();
    EXPECT_FALSE(result.has_value());
}

// ============================================================================
// Stream parser  (at::stream_parser — continuous serial-stream input)
//
// Per ITU-T V.250 §5.2.1 the default line terminator is <CR> (\r, S3=13).
// The stream_parser considers a command complete only when it finds \r.
//
// NOTE: string_view fields (raw, prefix, name) inside returned commands are
// views into the stream_parser's internal linear_buffer.  That buffer is
// partially erased after each recognised command, so those views must NOT be
// used after the next call to feed() or reset().  Only value-type fields
// (type, s_index, numeric params) are accessed here.
// ============================================================================

TEST(StreamParser, SingleCompleteCommand) {
    at::stream_parser sp;
    auto result = sp.feed("AT+CSQ\r");

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->size(), 1u);
    EXPECT_EQ(result->at(0).type, at::command_type::extended);
}

TEST(StreamParser, ReadCommandTerminatedByCR) {
    at::stream_parser sp;
    auto result = sp.feed("AT+COPS?\r");

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->size(), 1u);
    EXPECT_EQ(result->at(0).type, at::command_type::read);
}

TEST(StreamParser, IncompleteInputReturnsEmpty) {
    at::stream_parser sp;
    // No \r yet — command is not terminated.
    auto result = sp.feed("AT+CSQ");

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->empty());
}

TEST(StreamParser, FragmentedInputReassembly) {
    at::stream_parser sp;

    // First fragment — incomplete
    auto r1 = sp.feed("AT+CM");
    ASSERT_TRUE(r1.has_value());
    EXPECT_TRUE(r1->empty());

    // Second fragment — completes the command
    auto r2 = sp.feed("GF=1\r");
    ASSERT_TRUE(r2.has_value());
    ASSERT_EQ(r2->size(), 1u);
    EXPECT_EQ(r2->at(0).type, at::command_type::set);

    // Numeric parameter survives the consume
    auto val = r2->at(0).params.get_as<int64_t>(0);
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 1);
}

TEST(StreamParser, MultipleCommandsInOneFeed) {
    at::stream_parser sp;
    // Two complete commands in one buffer (realistic DTE burst)
    auto result = sp.feed("ATA\rATH\r");

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->size(), 2u);
    EXPECT_EQ(result->at(0).type, at::command_type::basic);
    EXPECT_EQ(result->at(1).type, at::command_type::basic);
}

TEST(StreamParser, SParameterSet) {
    at::stream_parser sp;
    auto result = sp.feed("ATS0=5\r");

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->size(), 1u);

    const auto& cmd = result->at(0);
    EXPECT_EQ(cmd.type, at::command_type::s_parameter);
    ASSERT_TRUE(cmd.s_index.has_value());
    EXPECT_EQ(*cmd.s_index, 0u);

    auto val = cmd.params.get_as<int64_t>(0);
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 5);
}

TEST(StreamParser, SParameterRead) {
    at::stream_parser sp;
    auto result = sp.feed("ATS7?\r");

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->size(), 1u);

    const auto& cmd = result->at(0);
    EXPECT_EQ(cmd.type, at::command_type::read);
    ASSERT_TRUE(cmd.s_index.has_value());
    EXPECT_EQ(*cmd.s_index, 7u);
}

TEST(StreamParser, ResetClearsBuffer) {
    at::stream_parser sp;

    // Feed a partial command, then reset
    (void)sp.feed("AT+CSQ");
    sp.reset();

    // After reset, the partial data is gone; a fresh command should parse
    auto result = sp.feed("ATZ\r");
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->size(), 1u);
    EXPECT_EQ(result->at(0).type, at::command_type::basic);
}

TEST(StreamParser, InvalidCommandReturnsError) {
    at::stream_parser sp;
    // Non-AT data terminated by \r must yield an error, not silently succeed.
    auto result = sp.feed("GARBAGE\r");
    EXPECT_FALSE(result.has_value());
}
