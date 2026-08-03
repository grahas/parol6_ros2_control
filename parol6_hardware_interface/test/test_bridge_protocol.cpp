// Copyright 2026 Graham Harison
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Wire-format tests for bridge_protocol.hpp -- the C++ side of a format
// that's duplicated (by necessity: one side is a real-time C++ control
// loop, the other an asyncio Python daemon) in
// parol6_bridge/parol6_bridge/protocol.py. Nothing here talks to a socket
// or needs a running bridge/robot; it only checks that the struct layout
// this process will send/receive bytes as still matches what protocol.py
// expects (same op codes, same sizes, same field order). See
// test_protocol.py on the Python side for the matching checks.
//
// This is the starting "hello world" test for this package -- the plugin's
// read()/write()/on_activate() lifecycle logic and BridgeClient's socket
// handling aren't covered yet; see README/HANDOFF for suggested next steps.

#include <gtest/gtest.h>

#include <cstddef>

#include "parol6_hardware_interface/bridge_protocol.hpp"

namespace parol6_hardware_interface
{

// Op codes must match protocol.py's OP_UPDATE/OP_ENABLE/OP_DISABLE/OP_PING
// exactly -- both sides put the raw integer on the wire, there's no named
// enum exchanged.
TEST(BridgeProtocol, OpCodesMatchPythonSide)
{
  EXPECT_EQ(kOpUpdate, 1);
  EXPECT_EQ(kOpEnable, 2);
  EXPECT_EQ(kOpDisable, 3);
  EXPECT_EQ(kOpPing, 4);
}

// Redundant with the static_asserts in bridge_protocol.hpp (those already
// fail the build if sizes drift), but kept as a runtime test too so this
// file has at least one meaningful assertion that shows up in `colcon
// test` output rather than only failing at compile time.
TEST(BridgeProtocol, StructSizesMatchPythonStructFormat)
{
  // REQUEST_FMT = "<B6d" in protocol.py: 1 (uint8) + 6*8 (double) = 49.
  EXPECT_EQ(sizeof(BridgeRequest), 49u);
  // RESPONSE_FMT = "<B6d6dBB": 1 + 6*8 + 6*8 + 1 + 1 = 99.
  EXPECT_EQ(sizeof(BridgeResponse), 99u);
}

// Struct sizes matching isn't sufficient on its own -- two same-size fields
// could still get silently swapped. #pragma pack(push, 1) in
// bridge_protocol.hpp means fields are laid out with no padding, in
// declaration order, so offsetof() pins down the exact byte layout that
// unpack_request()/pack_response() on the Python side assume.
TEST(BridgeProtocol, RequestFieldOffsetsMatchPackedLayout)
{
  EXPECT_EQ(offsetof(BridgeRequest, op), 0u);
  EXPECT_EQ(offsetof(BridgeRequest, target_pos_rad), 1u);
}

TEST(BridgeProtocol, ResponseFieldOffsetsMatchPackedLayout)
{
  EXPECT_EQ(offsetof(BridgeResponse, ok), 0u);
  EXPECT_EQ(offsetof(BridgeResponse, pos_rad), 1u);
  EXPECT_EQ(offsetof(BridgeResponse, vel_rad), 1u + 6 * sizeof(double));
  EXPECT_EQ(offsetof(BridgeResponse, hardware_connected), 1u + 12 * sizeof(double));
  EXPECT_EQ(offsetof(BridgeResponse, estop), 2u + 12 * sizeof(double));
}

}  // namespace parol6_hardware_interface
