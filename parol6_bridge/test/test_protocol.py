"""Wire-format tests for protocol.py -- the Python side of a format that's
duplicated (by necessity: one side is an asyncio daemon, the other a
real-time C++ control loop) in
parol6_hardware_interface/include/parol6_hardware_interface/bridge_protocol.hpp.
Nothing here opens a socket or needs a running bridge/robot; it only checks
that pack_response()/unpack_request() produce/consume bytes in the shape the
C++ side expects. See test_bridge_protocol.cpp on the C++ side for the
matching checks.

This is the starting "hello world" test for this package -- bridge_node.py's
actual asyncio dispatch/connection-handling logic isn't covered yet; see
README/HANDOFF for suggested next steps.
"""

import struct

from parol6_bridge.protocol import (
    OP_DISABLE,
    OP_ENABLE,
    OP_PING,
    OP_UPDATE,
    REQUEST_SIZE,
    RESPONSE_SIZE,
    pack_response,
    unpack_request,
)


def test_op_codes_match_cpp_side():
    # Must match kOpUpdate/kOpEnable/kOpDisable/kOpPing in bridge_protocol.hpp
    # exactly -- both sides put the raw integer on the wire, there's no
    # named enum exchanged.
    assert OP_UPDATE == 1
    assert OP_ENABLE == 2
    assert OP_DISABLE == 3
    assert OP_PING == 4


def test_struct_sizes_match_cpp_side():
    # sizeof(BridgeRequest) / sizeof(BridgeResponse) in bridge_protocol.hpp,
    # both static_assert'd there and re-checked at runtime in
    # test_bridge_protocol.cpp.
    assert REQUEST_SIZE == 49
    assert RESPONSE_SIZE == 99


def test_unpack_request_round_trip():
    target_pos_rad = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    raw = struct.pack("<B6d", OP_UPDATE, *target_pos_rad)

    op, parsed_pos = unpack_request(raw)

    assert op == OP_UPDATE
    assert parsed_pos == target_pos_rad


def test_pack_response_round_trip():
    pos_rad = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    vel_rad = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]

    raw = pack_response(
        ok=True,
        pos_rad=pos_rad,
        vel_rad=vel_rad,
        hardware_connected=True,
        estop=False,
    )

    assert len(raw) == RESPONSE_SIZE
    fields = struct.unpack("<B6d6dBB", raw)
    assert fields[0] == 1  # ok
    assert list(fields[1:7]) == pos_rad
    assert list(fields[7:13]) == vel_rad
    assert fields[13] == 1  # hardware_connected
    assert fields[14] == 0  # estop
