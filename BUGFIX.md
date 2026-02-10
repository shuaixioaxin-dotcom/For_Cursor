# ESP-NOW 通信故障分析与修复

## 故障现象

- **接收端**: `rx=0 ok=0 bad=0 test=0` — 完全没收到任何数据
- **发送端**: `tx_ok=0 tx_fail=0 queue_fail=0 attempt=0` — 从未尝试发送

## 根因分析

### 缺陷 1（致命）：WiFi 初始化序列错误导致发送端 `gEspNowReady = false`

**这是通信完全失败的根本原因。**

原代码的 `initEspNow()` 初始化序列：

```cpp
WiFi.mode(WIFI_STA);          // ① Arduino 库内部会调用 esp_wifi_start()
WiFi.setSleep(false);
WiFi.disconnect(true, true);
esp_wifi_set_ps(WIFI_PS_NONE);
gLastWifiStopErr = esp_wifi_stop();    // ② 手动停止 WiFi — 破坏了 ① 建立的状态
gLastWifiStartErr = esp_wifi_start();  // ③ 手动重启 WiFi — 状态可能不一致
gLastChannelErr = esp_wifi_set_channel(kEspNowChannel, ...); // ④ 信道设置可能失败
```

**问题链条**：
1. `WiFi.mode(WIFI_STA)` 通过 Arduino WiFi 库启动 WiFi，建立内部状态
2. `esp_wifi_stop()` 直接调用 ESP-IDF API 停止 WiFi 驱动，Arduino 库不知道这个操作
3. `esp_wifi_start()` 重新启动，但 Arduino WiFi 库的内部状态已经不一致
4. `esp_wifi_set_channel()` 在这种不一致状态下可能返回错误码

**在发送端**，`initEspNow()` 检查了信道设置结果：
```cpp
if (gLastChannelErr != ESP_OK) {
    return false;  // → gEspNowReady = false
}
```

**`gEspNowReady = false` 直接导致 `trySendPacket()` 在入口处返回**：
```cpp
void trySendPacket(...) {
    if (!gEspNowReady) {
        return;  // ← 永远走这里，attempt 始终为 0
    }
    // ...
    gTxAttemptCount++;  // ← 永远不会执行
}
```

这完美解释了日志中 `attempt=0` 的现象。

### 缺陷 2（中等）：广播 peer 信道设置为 0 可能不可靠

```cpp
peerInfo.channel = kUseBroadcastPeer ? 0 : kEspNowChannel;
```

`channel = 0` 表示"使用当前接口信道"。但如果 WiFi 接口信道因缺陷 1 而未正确设置，
广播也会发到错误的信道上。应显式指定信道以确保可靠性。

### 缺陷 3（轻微）：`WiFi.disconnect(true, true)` 擦除存储的凭据

`WiFi.disconnect(true, true)` 的第二个 `true` 参数会擦除 NVS 中存储的 WiFi 凭据。
对于 ESP-NOW 应用不需要这样做，改为 `WiFi.disconnect(false)` 即可。

### 缺陷 4（轻微）：发送端删除了无用的 `esp_now_deinit()` 调用

原代码在 `esp_now_init()` 之前调用 `esp_now_deinit()`，虽然不一定有害，
但首次初始化时 ESP-NOW 尚未 init 过，这个调用是多余的。

### 缺陷 5（调试改进）：缺少关键初始化状态日志

原代码在发送端 setup 日志中没有打印 `gEspNowReady` 的值，
导致排查时无法直接从日志看出 ESP-NOW 是否初始化成功。

## 修复内容

### 修复后的 `initEspNow()` 核心逻辑（发送端和接收端一致）：

```cpp
bool initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(false);          // 仅断开，不擦除凭据
    esp_wifi_set_ps(WIFI_PS_NONE);
    // 不再调用 esp_wifi_stop() / esp_wifi_start()
    delay(10);                        // 确保 WiFi 驱动就绪
    esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
    esp_now_init();
    // ...
}
```

### 修改摘要：

| 文件 | 修改 | 说明 |
|------|------|------|
| 接收端 `initEspNow()` | 移除 `esp_wifi_stop()/start()` | 修复 WiFi 状态不一致 |
| 接收端 `initEspNow()` | 移除 `esp_now_deinit()` | 移除多余调用 |
| 接收端 `initEspNow()` | 添加 `delay(10)` | 确保 WiFi 就绪 |
| 接收端 `initEspNow()` | `disconnect(true,true)` → `disconnect(false)` | 不擦除凭据 |
| 发送端 `initEspNow()` | 移除 `esp_wifi_stop()/start()` | **修复根本原因** |
| 发送端 `initEspNow()` | 移除 `esp_now_deinit()` | 移除多余调用 |
| 发送端 `initEspNow()` | 添加 `delay(10)` | 确保 WiFi 就绪 |
| 发送端 `initEspNow()` | `disconnect(true,true)` → `disconnect(false)` | 不擦除凭据 |
| 发送端 peer 设置 | `channel = 0` → `channel = kEspNowChannel` | 显式指定信道 |
| 发送端 setup 日志 | 添加 `espnow_ready` 打印 | 方便调试 |
| 发送端 initEspNow | 添加失败时的错误日志 | 方便定位问题 |

## 验证方法

刷写修复后的代码后，检查串口日志：

1. **发送端 setup 日志** 应显示：
   - `init=0 channel_err=0 peer_err=0`（全部为 ESP_OK）
   - `espnow_ready=1`
   - 不应出现 "ESP-NOW init failed."

2. **发送端周期日志** 应显示：
   - `attempt` 持续增长（不再为 0）
   - `tx_ok` 持续增长

3. **接收端周期日志** 应显示：
   - `rx` 和 `test` 持续增长（不再为 0）
   - `ack_ok` 持续增长（如果 ACK 启用）
