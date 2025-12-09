#!/bin/sh
#
# WiFi配置脚本 - 用于luckfox-buildroot系统
# 功能：修改wpa_supplicant.conf中的ssid和psk，并重新配置网络
#

# 配置文件路径
WPA_CONF="/etc/wpa_supplicant.conf"
WLAN_INTERFACE="wlan0"

# 显示使用帮助
usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -s, --ssid <WiFi名称>    设置WiFi SSID"
    echo "  -p, --psk <密码>         设置WiFi密码"
    echo "  -h, --help               显示帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 -s MyWiFi -p MyPassword123"
    echo "  $0 --ssid \"My WiFi\" --psk \"my password\""
    echo ""
    echo "如果不提供参数，将进入交互模式"
    exit 0
}

# 检查是否以root权限运行
check_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "错误: 请使用root权限运行此脚本"
        exit 1
    fi
}

# 备份配置文件
backup_config() {
    if [ -f "$WPA_CONF" ]; then
        cp "$WPA_CONF" "${WPA_CONF}.bak"
        echo "已备份原配置文件到 ${WPA_CONF}.bak"
    fi
}

# 更新配置文件
update_config() {
    local ssid="$1"
    local psk="$2"

    cat > "$WPA_CONF" << EOF
ctrl_interface=/var/run/wpa_supplicant
ap_scan=1
update_config=1

network={
        ssid="$ssid"
        psk="$psk"
        key_mgmt=WPA-PSK
}
EOF

    echo "配置文件已更新"
    echo "  SSID: $ssid"
    echo "  PSK: ********"
}

# 配置网络
configure_network() {
    echo ""
    echo "正在配置网络..."

    # 先停止可能运行的wpa_supplicant进程
    killall wpa_supplicant 2>/dev/null
    sleep 1

    # 启用无线接口
    echo "启用 $WLAN_INTERFACE 接口..."
    ip link set "$WLAN_INTERFACE" up
    if [ $? -ne 0 ]; then
        echo "警告: 启用 $WLAN_INTERFACE 失败"
    fi

    # 清理旧的socket文件
    echo "清理旧的wpa_supplicant socket文件..."
    if [ -e "/var/run/wpa_supplicant/$WLAN_INTERFACE" ]; then
        rm -f "/var/run/wpa_supplicant/$WLAN_INTERFACE"
    fi
    rm -f /var/run/wpa_supplicant/* 2>/dev/null

    # 确保目录存在
    mkdir -p /var/run/wpa_supplicant

    # 启动wpa_supplicant
    echo "启动wpa_supplicant..."
    wpa_supplicant -B -i "$WLAN_INTERFACE" -c "$WPA_CONF"
    if [ $? -eq 0 ]; then
        echo "wpa_supplicant 已启动"
    else
        echo "错误: wpa_supplicant 启动失败"
        exit 1
    fi

    # 等待连接
    echo "等待WiFi连接..."
    sleep 3

    # 获取IP地址（使用udhcpc，buildroot常用）
    echo "正在获取IP地址..."
    if command -v udhcpc >/dev/null 2>&1; then
        udhcpc -i "$WLAN_INTERFACE" -q -n
    elif command -v dhclient >/dev/null 2>&1; then
        dhclient "$WLAN_INTERFACE"
    else
        echo "提示: 请手动获取IP地址或配置静态IP"
    fi

    echo ""
    echo "网络配置完成！"
}

# 显示当前配置
show_current_config() {
    if [ -f "$WPA_CONF" ]; then
        echo "当前配置:"
        echo "----------------------------------------"
        cat "$WPA_CONF"
        echo "----------------------------------------"
        echo ""
    fi
}

# 检查网络状态
check_status() {
    echo ""
    echo "网络状态:"
    echo "----------------------------------------"
    ip addr show "$WLAN_INTERFACE" 2>/dev/null || echo "$WLAN_INTERFACE 接口不存在"
    echo "----------------------------------------"
}

# 主函数
main() {
    local ssid=""
    local psk=""

    # 解析命令行参数
    while [ $# -gt 0 ]; do
        case "$1" in
            -s|--ssid)
                ssid="$2"
                shift 2
                ;;
            -p|--psk)
                psk="$2"
                shift 2
                ;;
            -h|--help)
                usage
                ;;
            *)
                echo "未知参数: $1"
                usage
                ;;
        esac
    done

    # 检查root权限
    check_root

    # 显示当前配置
    show_current_config

    # 如果没有提供参数，进入交互模式
    if [ -z "$ssid" ] || [ -z "$psk" ]; then
        echo "进入交互模式..."
        echo ""
        
        if [ -z "$ssid" ]; then
            printf "请输入WiFi名称(SSID): "
            read ssid
        fi
        
        if [ -z "$psk" ]; then
            printf "请输入WiFi密码(PSK): "
            read psk
        fi
    fi

    # 验证输入
    if [ -z "$ssid" ]; then
        echo "错误: SSID不能为空"
        exit 1
    fi

    if [ -z "$psk" ]; then
        echo "错误: 密码不能为空"
        exit 1
    fi

    if [ ${#psk} -lt 8 ]; then
        echo "错误: WPA密码长度必须至少8个字符"
        exit 1
    fi

    # 备份并更新配置
    backup_config
    update_config "$ssid" "$psk"

    # 配置网络
    configure_network

    # 显示状态
    check_status
}

# 运行主函数
main "$@"
