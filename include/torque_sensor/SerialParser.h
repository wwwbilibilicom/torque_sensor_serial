#pragma once

#include <vector>
#include <cstdint>
#include <functional>

// 针对新协议定义的结构体
struct SensorFrame {
    int16_t value; // 假设 2 字节数据段代表一个 16 位传感器值
};

class SerialParser {
public:
    using FrameCallback = std::function<void(const SensorFrame&)>;

    SerialParser(FrameCallback callback);

    // 处理串口流入的原始字节
    void parse(const uint8_t* buffer, size_t length);

private:
    std::vector<uint8_t> ring_buffer;
    FrameCallback on_frame_parsed;



    // --- 协议配置区 ---
    const uint8_t FRAME_HEAD = 0x03; // 帧头由 0xFEF5 改为 0x04
    const uint8_t FRAME_TAIL = 0x0d; // 保持 0x0d，如果没有帧尾可设为 0

    // 长度计算：头(1) + 数据(2) + 尾(1) = 4 字节
    const size_t TOTAL_FRAME_LEN = 4; 
    const size_t DATA_OFFSET = 1;    // 数据段从第 2 个字节开始
    // ----------------

    void try_parse_buffer();
};
