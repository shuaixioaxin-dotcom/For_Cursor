#ifndef CONFIG_H
#define CONFIG_H

// ================= 性能优化配置指南 =================
//
// 本文件提供了可调参数，用于优化系统达到200Hz采集频率
//

// ================= 批量输出配置 =================
// BATCH_SIZE: 每批收集的完整样本数
//   - 值越大：输出频率越低，但采集频率越高
//   - 值越小：输出更频繁，但可能影响采集速度
//   - 推荐值：4-8 (200Hz采集时，输出频率25-50Hz)
//
// 示例：
//   BATCH_SIZE=4  -> 200Hz采集, 50Hz输出 (每次输出8个IMU数据点)
//   BATCH_SIZE=8  -> 200Hz采集, 25Hz输出 (每次输出16个IMU数据点)
#ifndef BATCH_SIZE
#define BATCH_SIZE 4
#endif

// OUTPUT_BINARY: 输出格式选择
//   - true:  二进制输出，更高效，适合高速数据传输
//   - false: 文本输出，便于调试和人类阅读
#ifndef OUTPUT_BINARY
#define OUTPUT_BINARY false
#endif

// ================= 时序优化参数 =================
// 这些参数已针对200Hz进行优化，通常不需要修改

// RESPONSE_TIMEOUT_MS: Modbus响应超时时间(毫秒)
//   - 降低此值可以加快错误恢复
//   - 但不应低于实际响应时间，否则会增加误判
//   - 推荐范围: 10-20ms
#ifndef RESPONSE_TIMEOUT_MS
#define RESPONSE_TIMEOUT_MS 15
#endif

// BUS_SILENCE_US: 总线静默时间(微秒)
//   - RS485总线发送前等待时间
//   - 降低可提升速度，但可能导致冲突
//   - 推荐范围: 100-500us
#ifndef BUS_SILENCE_US
#define BUS_SILENCE_US 200
#endif

// TX_ENABLE_DELAY_US: 发送使能延迟(微秒)
//   - RS485芯片从接收切换到发送的延迟
//   - 根据硬件特性调整
#ifndef TX_ENABLE_DELAY_US
#define TX_ENABLE_DELAY_US 20
#endif

// TX_DISABLE_DELAY_US: 发送禁用延迟(微秒)
//   - 发送完成后切换回接收的延迟
#ifndef TX_DISABLE_DELAY_US
#define TX_DISABLE_DELAY_US 30
#endif

// ================= 性能监控配置 =================
// ENABLE_STATS: 启用性能统计输出
//   - true:  每秒输出实际采集频率（用于调试）
//   - false: 禁用统计输出（用于生产环境）
#ifndef ENABLE_STATS
#define ENABLE_STATS false
#endif

// ================= 高级配置 =================
// 以下参数通常不需要修改

// RX_RING_SIZE: 接收环形缓冲区大小
//   - 必须足够容纳至少一个完整响应帧
//   - 更大的缓冲区可以处理突发数据
#ifndef RX_RING_SIZE
#define RX_RING_SIZE 128
#endif

// ================= 使用示例 =================
//
// 1. 文本模式，批量大小4，用于调试：
//    #define BATCH_SIZE 4
//    #define OUTPUT_BINARY false
//    #define ENABLE_STATS true
//
// 2. 二进制模式，批量大小8，用于生产：
//    #define BATCH_SIZE 8
//    #define OUTPUT_BINARY true
//    #define ENABLE_STATS false
//
// 3. 极限性能模式（需要高性能接收端）：
//    #define BATCH_SIZE 2
//    #define OUTPUT_BINARY true
//    #define RESPONSE_TIMEOUT_MS 10
//    #define BUS_SILENCE_US 100
//

#endif // CONFIG_H
