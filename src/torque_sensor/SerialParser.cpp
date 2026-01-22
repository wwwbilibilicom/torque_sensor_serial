#include "torque_sensor/SerialParser.h"
#include <algorithm>

SerialParser::SerialParser(FrameCallback callback) 
    : on_frame_parsed(callback) {
    ring_buffer.reserve(512); 
}

void SerialParser::parse(const uint8_t* buffer, size_t length) {
    ring_buffer.insert(ring_buffer.end(), buffer, buffer + length);
    try_parse_buffer();
}

void SerialParser::try_parse_buffer() {
    // 只要缓冲区够一个完整包
    while (ring_buffer.size() >= TOTAL_FRAME_LEN) {
        
        // 1. 寻找帧头 0xFF
        if (ring_buffer[0] != FRAME_HEAD) {
            ring_buffer.erase(ring_buffer.begin());
            continue;
        }

        // 2. 验证帧尾 (如果有帧尾逻辑)
        if (ring_buffer[TOTAL_FRAME_LEN - 1] != FRAME_TAIL) {
            // 如果头对了但尾不对，说明这个 0xFF 可能是数据里的值，不是真帧头
            ring_buffer.erase(ring_buffer.begin());
            continue;
        }

        // 3. 解析 2 字节数据段
        // 假设是大端序 (High Byte first)，如果是小端序则反过来
        int16_t raw_val = static_cast<int16_t>((ring_buffer[DATA_OFFSET]) | 
                                                ring_buffer[DATA_OFFSET + 1]<<8);

        // 4. 执行回调
        if (on_frame_parsed) {
            SensorFrame frame;
            frame.value = raw_val;
            on_frame_parsed(frame);
        }

        // 5. 弹出已处理数据
        ring_buffer.erase(ring_buffer.begin(), ring_buffer.begin() + TOTAL_FRAME_LEN);
    }
}