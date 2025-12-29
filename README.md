# USB设备udev规则生成工具

这是一个用于管理USB串口设备udev规则的脚本工具。

## 功能特性

1. **列出USB设备** - 显示当前所有接入的ttyUSB设备
2. **查看设备信息** - 获取设备的详细信息（Vendor ID, Product ID, Serial Number等）
3. **生成udev规则** - 根据设备信息生成udev规则
   - **自定义规则文件名** - 可自行命名规则文件（如 `99-my-serial.rules`）
   - **自定义符号链接名** - 可自行定义设备符号链接（如 `serial_port_1`）
4. **安装规则文件** - 将规则文件拷贝到 `/etc/udev/rules.d/`

## 使用方法

### 交互式菜单模式

```bash
./usb_udev_rules.sh
```

### 命令行模式

```bash
# 列出所有USB串口设备
./usb_udev_rules.sh -l

# 查看指定设备的详细信息
./usb_udev_rules.sh -i /dev/ttyUSB0

# 交互式生成规则（可自定义文件名和符号链接）
./usb_udev_rules.sh -g

# 为所有设备生成规则（可自定义文件名和符号链接）
./usb_udev_rules.sh -a

# 生成规则并安装到系统
./usb_udev_rules.sh -a -s

# 安装已有的规则文件
./usb_udev_rules.sh -s

# 显示帮助信息
./usb_udev_rules.sh -h
```

## 使用流程示例

1. 运行脚本：`./usb_udev_rules.sh -a`
2. 输入规则文件名：`99-my-usb-devices.rules`
3. 为每个设备输入符号链接名：`serial_port_1`、`serial_port_2` 等
4. 使用 `-s` 安装规则到系统

## 生成的规则示例

```
# Device: /dev/ttyUSB0
# Manufacturer: FTDI
# Product: FT232R USB UART
# Symlink: /dev/serial_port_1
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="FTAI6KDH", MODE="0666", SYMLINK+="serial_port_1"
```

规则说明：
- `SUBSYSTEM=="tty"` - 匹配tty子系统
- `ATTRS{idVendor}` - USB厂商ID
- `ATTRS{idProduct}` - USB产品ID
- `ATTRS{serial}` - 设备序列号（用于区分同型号的多个设备）
- `MODE="0666"` - 设置设备权限为所有用户可读写
- `SYMLINK+="serial_port_1"` - 创建固定的符号链接 `/dev/serial_port_1`

## 安装后操作

规则安装后，脚本会自动执行以下命令使规则生效：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

如果规则未生效，可以尝试重新插拔USB设备。

## 手动查看USB设备信息

```bash
# 查看设备路径
udevadm info -q path -n /dev/ttyUSB0

# 查看设备完整属性
udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0)
```

## 注意事项

1. 安装规则需要root权限（sudo）
2. 规则文件会存放在 `/etc/udev/rules.d/` 目录下
3. 序列号是区分同型号多设备的关键，确保设备有唯一序列号
4. 符号链接名称建议使用有意义的名称，便于识别设备用途
