// ===============================================
// Modbus IMU 频率优化配置预设
// ===============================================
// 使用方法：根据您的需求取消注释其中一个配置

#ifndef CONFIG_PRESETS_H
#define CONFIG_PRESETS_H

// ============================================
// 预设1：保守配置 (稳定优先，150-180Hz)
// ============================================
// 适用场景：
// - 电缆长度 > 5米
// - 环境干扰较大
// - 首次部署，需要确保稳定性

// #define CONFIG_CONSERVATIVE
#ifdef CONFIG_CONSERVATIVE
  #define RESPONSE_TIMEOUT_MS    8
  #define BUS_SILENCE_US         200
  #define TX_ENABLE_DELAY_US     15
  #define TX_DISABLE_DELAY_US    30
  #define FAIL_COOLDOWN_MS       8
  #define MAX_BACKOFF_SHIFT      3
#endif

// ============================================
// 预设2：平衡配置 (推荐，180-220Hz)
// ============================================
// 适用场景：
// - 电缆长度 < 5米
// - 标准工业环境
// - 已测试过通信稳定性

#define CONFIG_BALANCED  // 默认推荐配置
#ifdef CONFIG_BALANCED
  #define RESPONSE_TIMEOUT_MS    5
  #define BUS_SILENCE_US         100
  #define TX_ENABLE_DELAY_US     10
  #define TX_DISABLE_DELAY_US    20
  #define FAIL_COOLDOWN_MS       5
  #define MAX_BACKOFF_SHIFT      2
#endif

// ============================================
// 预设3：激进配置 (性能优先，220-250Hz)
// ============================================
// 适用场景：
// - 电缆长度 < 2米
// - 低干扰环境（实验室）
// - 高质量屏蔽电缆
// - 需要极限性能

// #define CONFIG_AGGRESSIVE
#ifdef CONFIG_AGGRESSIVE
  #define RESPONSE_TIMEOUT_MS    3
  #define BUS_SILENCE_US         50
  #define TX_ENABLE_DELAY_US     5
  #define TX_DISABLE_DELAY_US    10
  #define FAIL_COOLDOWN_MS       3
  #define MAX_BACKOFF_SHIFT      2
#endif

// ============================================
// 预设4：超高速配置 (需要2Mbps波特率)
// ============================================
// 适用场景：
// - IMU支持2Mbps波特率
// - 高质量短距离电缆 (< 1米)
// - 实验室环境

// #define CONFIG_ULTRA_FAST
#ifdef CONFIG_ULTRA_FAST
  #define RS485_BAUD_OVERRIDE    2000000
  #define RESPONSE_TIMEOUT_MS    3
  #define BUS_SILENCE_US         30
  #define TX_ENABLE_DELAY_US     3
  #define TX_DISABLE_DELAY_US    8
  #define FAIL_COOLDOWN_MS       2
  #define MAX_BACKOFF_SHIFT      2
#endif

// ============================================
// 预设5：调试配置 (便于问题诊断)
// ============================================
// 适用场景：
// - 排查通信问题
// - 测量实际响应时间
// - 验证硬件连接

// #define CONFIG_DEBUG
#ifdef CONFIG_DEBUG
  #define RESPONSE_TIMEOUT_MS    50
  #define BUS_SILENCE_US         500
  #define TX_ENABLE_DELAY_US     30
  #define TX_DISABLE_DELAY_US    60
  #define FAIL_COOLDOWN_MS       100
  #define MAX_BACKOFF_SHIFT      0  // 禁用退避
  #define ENABLE_DEBUG_OUTPUT       // 启用调试输出
#endif

// ============================================
// 自定义配置示例
// ============================================
// 如果预设不满足需求，可以自定义：

// #define CONFIG_CUSTOM
#ifdef CONFIG_CUSTOM
  #define RESPONSE_TIMEOUT_MS    6     // 根据实测调整
  #define BUS_SILENCE_US         120   // 根据实测调整
  #define TX_ENABLE_DELAY_US     12    // 根据实测调整
  #define TX_DISABLE_DELAY_US    25    // 根据实测调整
  #define FAIL_COOLDOWN_MS       6     // 根据实测调整
  #define MAX_BACKOFF_SHIFT      2     // 根据实测调整
#endif

// ============================================
// 配置检查
// ============================================
#ifndef RESPONSE_TIMEOUT_MS
  #error "请至少选择一个配置预设"
#endif

// 时序合理性检查
#if RESPONSE_TIMEOUT_MS < 2
  #warning "RESPONSE_TIMEOUT_MS太小，可能导致超时误判"
#endif

#if BUS_SILENCE_US < 40
  #warning "BUS_SILENCE_US小于Modbus RTU最小要求(3.5字符时间)"
#endif

// ============================================
// 配置信息输出
// ============================================
inline void printActiveConfig() {
  Serial.println("# ===== 当前配置 =====");
  
  #ifdef CONFIG_CONSERVATIVE
    Serial.println("# 配置模式: 保守配置");
  #elif defined(CONFIG_BALANCED)
    Serial.println("# 配置模式: 平衡配置 (推荐)");
  #elif defined(CONFIG_AGGRESSIVE)
    Serial.println("# 配置模式: 激进配置");
  #elif defined(CONFIG_ULTRA_FAST)
    Serial.println("# 配置模式: 超高速配置");
  #elif defined(CONFIG_DEBUG)
    Serial.println("# 配置模式: 调试配置");
  #elif defined(CONFIG_CUSTOM)
    Serial.println("# 配置模式: 自定义配置");
  #endif
  
  Serial.print("# RESPONSE_TIMEOUT_MS: ");
  Serial.println(RESPONSE_TIMEOUT_MS);
  
  Serial.print("# BUS_SILENCE_US: ");
  Serial.println(BUS_SILENCE_US);
  
  Serial.print("# TX_ENABLE_DELAY_US: ");
  Serial.println(TX_ENABLE_DELAY_US);
  
  Serial.print("# TX_DISABLE_DELAY_US: ");
  Serial.println(TX_DISABLE_DELAY_US);
  
  Serial.print("# FAIL_COOLDOWN_MS: ");
  Serial.println(FAIL_COOLDOWN_MS);
  
  Serial.print("# MAX_BACKOFF_SHIFT: ");
  Serial.println(MAX_BACKOFF_SHIFT);
  
  Serial.println("# ====================");
}

#endif // CONFIG_PRESETS_H
