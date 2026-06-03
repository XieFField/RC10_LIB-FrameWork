#include "Module_Serial1Protocol.h"
#include "BSP_TimeStamp.h"

#include <array>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <windows.h>

namespace
{
std::ofstream g_log("jia_docs/tests/legacy_ai_tests/serial1_protocol_host/build/last_run.log", std::ios::trunc);

uint8_t calculateCheck(const std::array<uint8_t, 3> &payload)
{
    const uint16_t sum = payload[0] + payload[1] + payload[2];
    return static_cast<uint8_t>((~((sum * sum) & 0xFF)) & SERIAL1_CHECKSUM_MASK);
}

std::array<uint8_t, SERIAL1_FRAME_LEN> buildFrame(const std::array<uint8_t, 3> &payload, uint8_t parity)
{
    std::array<uint8_t, SERIAL1_FRAME_LEN> frame{};
    frame[0] = SERIAL1_FRAME_HEAD0;
    frame[1] = SERIAL1_FRAME_HEAD1;
    frame[2] = payload[0];
    frame[3] = payload[1];
    frame[4] = payload[2];
    frame[5] = static_cast<uint8_t>(calculateCheck(payload) | ((parity & 0x01U) << 6));
    frame[6] = SERIAL1_FRAME_TAIL0;
    frame[7] = SERIAL1_FRAME_TAIL1;
    return frame;
}

void require(bool condition, const char *message)
{
    if (!condition) {
        g_log << "FAIL: " << message << std::endl;
        std::cerr << "FAIL: " << message << std::endl;
        std::exit(1);
    }
    g_log << "OK: " << message << std::endl;
}

bool isAckFrame(const std::vector<uint8_t> &frame)
{
    if (frame.size() != SERIAL1_FRAME_LEN) {
        return false;
    }
    return frame[2] == 0x00 && frame[3] == 0x00 && frame[4] == 0x00;
}

uint8_t extractParity(const std::vector<uint8_t> &frame)
{
    return static_cast<uint8_t>((frame[5] & SERIAL1_PARITY_BIT_MASK) >> 6);
}
}

int main()
{
    g_log << "start" << std::endl;
    UART_HandleTypeDef uart{};
    auto &protocol = Serial1Protocol::getInstance();

    testHostResetHalState();
    g_test_time_ms = 0;
    protocol.init(&uart);

    require(g_test_receive_to_idle_calls == 1, "init must arm ReceiveToIdle DMA exactly once");
    require(g_test_last_receive_uart == &uart, "init must arm DMA on the provided UART");
    require(g_test_last_receive_to_idle_len == 30, "init must use the 30-byte RX buffer");
    require(g_test_clear_idle_calls == 1, "init must clear the UART idle flag");

    DataPacket_t packet{};

    testHostResetHalState();
    auto cmdFrame = buildFrame({0x00, 0x00, 0x33}, 0);
    protocol.onUartReceive(cmdFrame.data(), static_cast<uint16_t>(cmdFrame.size()));
    require(protocol.getLatestData(&packet), "valid CMD frame must be surfaced as latest data");
    require(packet.type == DATA_TYPE_CMD, "CMD frame must decode as DATA_TYPE_CMD");
    require(packet.data.cmd == 0x33, "CMD frame must preserve the outbound command byte");
    require(g_test_receive_to_idle_calls == 1, "processing a frame must re-arm ReceiveToIdle DMA");
    require(!g_test_uart_tx_frames.empty(), "processing a business frame must send an ACK");
    require(isAckFrame(g_test_uart_tx_frames.back()), "business frame ACK must carry the all-zero payload");

    testHostResetHalState();
    protocol.onUartReceive(cmdFrame.data(), static_cast<uint16_t>(cmdFrame.size()));
    require(!protocol.getLatestData(&packet), "same payload with same parity must be deduplicated");
    require(!g_test_uart_tx_frames.empty(), "duplicate frame must still be ACKed");
    require(isAckFrame(g_test_uart_tx_frames.back()), "duplicate frame ACK must keep the all-zero payload");

    testHostResetHalState();
    auto cmdFrameParityFlip = buildFrame({0x00, 0x00, 0x33}, 1);
    protocol.onUartReceive(cmdFrameParityFlip.data(), static_cast<uint16_t>(cmdFrameParityFlip.size()));
    require(protocol.getLatestData(&packet), "same payload with flipped parity must be treated as fresh data");
    require(packet.type == DATA_TYPE_CMD && packet.data.cmd == 0x33, "flipped-parity CMD must preserve decoded data");

    testHostResetHalState();
    auto packedKfsFrame = buildFrame({0x12, 0x30, 0x00}, 0);
    protocol.onUartReceive(packedKfsFrame.data(), static_cast<uint16_t>(packedKfsFrame.size()));
    require(protocol.getLatestData(&packet), "packed KFS frame must be surfaced as latest data");
    require(packet.type == DATA_TYPE_KFS, "packed KFS frame must decode as DATA_TYPE_KFS");
    require(packet.data.kfs[0] == 0x01 && packet.data.kfs[1] == 0x02 && packet.data.kfs[2] == 0x03,
        "packed KFS frame must unpack three 4-bit positions");

    testHostResetHalState();
    protocol.send_cmd_to_R2(0x44);
    require(!g_test_uart_tx_frames.empty(), "send_cmd_to_R2 must emit UART traffic");
    const uint8_t firstCommandParity = extractParity(g_test_uart_tx_frames.front());
    require(g_test_uart_tx_frames.front()[2] == 0x00 && g_test_uart_tx_frames.front()[3] == 0x00 &&
            g_test_uart_tx_frames.front()[4] == 0x44,
        "send_cmd_to_R2 must place the command in the third payload byte");

    testHostResetHalState();
    protocol.send_cmd_to_R2(0x44);
    require(!g_test_uart_tx_frames.empty(), "sending the same command twice must still emit UART traffic");
    const uint8_t secondCommandParity = extractParity(g_test_uart_tx_frames.front());
    require(firstCommandParity != secondCommandParity, "re-sending the same command must flip the parity bit");

    testHostResetHalState();
    protocol.R1_Send_KFS(0x01, 0x02, 0x03);
    require(!g_test_uart_tx_frames.empty(), "R1_Send_KFS must emit UART traffic");
    require(g_test_uart_tx_frames.front()[2] == 0x12 && g_test_uart_tx_frames.front()[3] == 0x30 &&
            g_test_uart_tx_frames.front()[4] == 0x00,
        "R1_Send_KFS must keep the current three-nibble packing contract");

    g_log << "PASS" << std::endl;
    g_log.flush();
    ::TerminateProcess(::GetCurrentProcess(), 0);
}
