#!/bin/bash

#===============================================================================
# USB设备udev规则生成脚本
# 功能：
#   1. 列出当前所有ttyUSB设备
#   2. 查看设备详细信息（Vendor ID, Product ID, Serial Number等）
#   3. 根据设备信息生成udev规则
#   4. 将规则文件拷贝到 /etc/udev/rules.d/
#===============================================================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 规则文件存放目录
RULES_DIR="/etc/udev/rules.d"
# 本地生成的规则文件（默认值，可由用户自定义）
LOCAL_RULES_FILE=""

#-------------------------------------------------------------------------------
# 打印带颜色的消息
#-------------------------------------------------------------------------------
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

#-------------------------------------------------------------------------------
# 获取用户自定义的规则文件名
#-------------------------------------------------------------------------------
get_rules_filename() {
    echo ""
    print_info "请输入规则文件名称"
    echo "  - 建议以数字开头（如99-xxx.rules），数字越大优先级越高"
    echo "  - 必须以 .rules 结尾"
    echo "  - 示例: 99-usb-serial.rules, 99-my-devices.rules"
    echo ""
    
    while true; do
        read -p "规则文件名 [默认: 99-usb-serial.rules]: " input_name
        
        # 如果用户直接回车，使用默认值
        if [ -z "$input_name" ]; then
            LOCAL_RULES_FILE="99-usb-serial.rules"
            break
        fi
        
        # 检查是否以 .rules 结尾
        if [[ "$input_name" == *.rules ]]; then
            LOCAL_RULES_FILE="$input_name"
            break
        else
            print_warning "文件名必须以 .rules 结尾，请重新输入"
        fi
    done
    
    print_success "规则文件名设置为: $LOCAL_RULES_FILE"
}

#-------------------------------------------------------------------------------
# 列出所有ttyUSB设备
#-------------------------------------------------------------------------------
list_usb_devices() {
    echo ""
    echo "=============================================="
    echo "        当前接入的USB串口设备列表"
    echo "=============================================="
    
    local devices=$(ls /dev/ttyUSB* 2>/dev/null)
    
    if [ -z "$devices" ]; then
        print_warning "未检测到任何ttyUSB设备"
        return 1
    fi
    
    echo ""
    for dev in $devices; do
        echo "  - $dev"
    done
    echo ""
    return 0
}

#-------------------------------------------------------------------------------
# 获取设备详细信息
#-------------------------------------------------------------------------------
get_device_info() {
    local device=$1
    
    if [ ! -e "$device" ]; then
        print_error "设备 $device 不存在"
        return 1
    fi
    
    echo ""
    echo "=============================================="
    echo "        设备详细信息: $device"
    echo "=============================================="
    echo ""
    
    # 获取设备路径
    local dev_path=$(udevadm info -q path -n "$device" 2>/dev/null)
    
    if [ -z "$dev_path" ]; then
        print_error "无法获取设备路径"
        return 1
    fi
    
    # 显示完整的设备信息
    print_info "执行命令: udevadm info -a -p $dev_path"
    echo ""
    udevadm info -a -p "$dev_path"
    
    return 0
}

#-------------------------------------------------------------------------------
# 提取设备的关键属性
#-------------------------------------------------------------------------------
extract_device_attrs() {
    local device=$1
    
    local dev_path=$(udevadm info -q path -n "$device" 2>/dev/null)
    
    if [ -z "$dev_path" ]; then
        return 1
    fi
    
    # 获取设备属性
    local info=$(udevadm info -a -p "$dev_path" 2>/dev/null)
    
    # 提取 idVendor
    ID_VENDOR=$(echo "$info" | grep -m1 'ATTRS{idVendor}' | sed 's/.*=="\([^"]*\)".*/\1/')
    
    # 提取 idProduct
    ID_PRODUCT=$(echo "$info" | grep -m1 'ATTRS{idProduct}' | sed 's/.*=="\([^"]*\)".*/\1/')
    
    # 提取 serial
    SERIAL=$(echo "$info" | grep -m1 'ATTRS{serial}' | sed 's/.*=="\([^"]*\)".*/\1/')
    
    # 提取 manufacturer (可选)
    MANUFACTURER=$(echo "$info" | grep -m1 'ATTRS{manufacturer}' | sed 's/.*=="\([^"]*\)".*/\1/')
    
    # 提取 product (可选)
    PRODUCT=$(echo "$info" | grep -m1 'ATTRS{product}' | sed 's/.*=="\([^"]*\)".*/\1/')
    
    return 0
}

#-------------------------------------------------------------------------------
# 显示提取的设备属性
#-------------------------------------------------------------------------------
show_device_attrs() {
    local device=$1
    
    extract_device_attrs "$device"
    
    echo ""
    echo "----------------------------------------------"
    echo "  设备关键属性 ($device)"
    echo "----------------------------------------------"
    echo "  Vendor ID    : ${ID_VENDOR:-未知}"
    echo "  Product ID   : ${ID_PRODUCT:-未知}"
    echo "  Serial Number: ${SERIAL:-未知}"
    echo "  Manufacturer : ${MANUFACTURER:-未知}"
    echo "  Product      : ${PRODUCT:-未知}"
    echo "----------------------------------------------"
    echo ""
}

#-------------------------------------------------------------------------------
# 生成单个设备的udev规则
#-------------------------------------------------------------------------------
generate_single_rule() {
    local device=$1
    local symlink_name=$2
    
    extract_device_attrs "$device"
    
    if [ -z "$ID_VENDOR" ] || [ -z "$ID_PRODUCT" ]; then
        print_error "无法获取设备的Vendor ID或Product ID"
        return 1
    fi
    
    local rule=""
    
    # 构建规则
    rule="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$ID_VENDOR\", ATTRS{idProduct}==\"$ID_PRODUCT\""
    
    # 如果有序列号，添加序列号匹配
    if [ -n "$SERIAL" ]; then
        rule="$rule, ATTRS{serial}==\"$SERIAL\""
    fi
    
    # 添加权限和符号链接
    rule="$rule, MODE=\"0666\""
    
    if [ -n "$symlink_name" ]; then
        rule="$rule, SYMLINK+=\"$symlink_name\""
    fi
    
    echo "$rule"
}

#-------------------------------------------------------------------------------
# 交互式生成规则
#-------------------------------------------------------------------------------
interactive_generate_rules() {
    local devices=$(ls /dev/ttyUSB* 2>/dev/null)
    
    if [ -z "$devices" ]; then
        print_warning "未检测到任何ttyUSB设备"
        return 1
    fi
    
    # 获取用户自定义的规则文件名
    get_rules_filename
    
    local rules_content=""
    local rules_header="# USB Serial Port udev Rules\n"
    rules_header+="# 自动生成时间: $(date)\n"
    rules_header+="# \n"
    rules_header+="# 用法:\n"
    rules_header+="#   1. 将此文件拷贝到 /etc/udev/rules.d/\n"
    rules_header+="#   2. 重新加载udev规则: sudo udevadm control --reload-rules\n"
    rules_header+="#   3. 触发规则: sudo udevadm trigger\n"
    rules_header+="#\n\n"
    
    rules_content="$rules_header"
    
    echo ""
    echo "=============================================="
    echo "        交互式规则生成"
    echo "=============================================="
    
    for dev in $devices; do
        echo ""
        show_device_attrs "$dev"
        
        read -p "是否为设备 $dev 生成规则? (y/n): " choice
        
        if [ "$choice" = "y" ] || [ "$choice" = "Y" ]; then
            echo ""
            echo "  请输入符号链接名称 (SYMLINK)"
            echo "  - 这将在 /dev/ 下创建固定名称的设备链接"
            echo "  - 例如输入 'serial_port_1' 将创建 /dev/serial_port_1"
            echo "  - 留空则不创建符号链接"
            echo ""
            read -p "  符号链接名称: " symlink
            
            local rule=$(generate_single_rule "$dev" "$symlink")
            
            if [ -n "$rule" ]; then
                # 添加注释
                rules_content+="# Device: $dev\n"
                if [ -n "$MANUFACTURER" ]; then
                    rules_content+="# Manufacturer: $MANUFACTURER\n"
                fi
                if [ -n "$PRODUCT" ]; then
                    rules_content+="# Product: $PRODUCT\n"
                fi
                rules_content+="$rule\n\n"
                
                print_success "规则已添加"
            fi
        fi
    done
    
    # 保存规则文件
    echo -e "$rules_content" > "$LOCAL_RULES_FILE"
    print_success "规则文件已保存到: $(pwd)/$LOCAL_RULES_FILE"
    
    echo ""
    echo "----------------------------------------------"
    echo "  生成的规则文件内容:"
    echo "----------------------------------------------"
    cat "$LOCAL_RULES_FILE"
    echo "----------------------------------------------"
    
    return 0
}

#-------------------------------------------------------------------------------
# 自动为所有设备生成规则（需要用户输入符号链接名）
#-------------------------------------------------------------------------------
auto_generate_rules() {
    local devices=$(ls /dev/ttyUSB* 2>/dev/null)
    
    if [ -z "$devices" ]; then
        print_warning "未检测到任何ttyUSB设备"
        return 1
    fi
    
    # 获取用户自定义的规则文件名
    get_rules_filename
    
    local rules_content=""
    local rules_header="# USB Serial Port udev Rules\n"
    rules_header+="# 自动生成时间: $(date)\n"
    rules_header+="# 规则文件: $LOCAL_RULES_FILE\n"
    rules_header+="# \n"
    rules_header+="# 用法:\n"
    rules_header+="#   1. 将此文件拷贝到 /etc/udev/rules.d/\n"
    rules_header+="#   2. 重新加载udev规则: sudo udevadm control --reload-rules\n"
    rules_header+="#   3. 触发规则: sudo udevadm trigger\n"
    rules_header+="#\n\n"
    
    rules_content="$rules_header"
    
    echo ""
    echo "=============================================="
    echo "        为所有设备生成规则"
    echo "=============================================="
    
    local count=0
    for dev in $devices; do
        extract_device_attrs "$dev"
        
        if [ -n "$ID_VENDOR" ] && [ -n "$ID_PRODUCT" ]; then
            # 显示设备信息
            echo ""
            echo "----------------------------------------------"
            echo "  设备: $dev"
            echo "  Vendor ID    : $ID_VENDOR"
            echo "  Product ID   : $ID_PRODUCT"
            echo "  Serial Number: ${SERIAL:-无}"
            echo "----------------------------------------------"
            
            # 让用户输入符号链接名称
            echo ""
            echo "  请输入此设备的符号链接名称 (SYMLINK)"
            echo "  - 这将在 /dev/ 下创建固定名称的设备链接"
            echo "  - 例如输入 'serial_port_1' 将创建 /dev/serial_port_1"
            echo "  - 留空则不创建符号链接"
            echo ""
            read -p "  符号链接名称: " symlink
            
            local rule=$(generate_single_rule "$dev" "$symlink")
            
            if [ -n "$rule" ]; then
                rules_content+="# Device: $dev\n"
                if [ -n "$MANUFACTURER" ]; then
                    rules_content+="# Manufacturer: $MANUFACTURER\n"
                fi
                if [ -n "$PRODUCT" ]; then
                    rules_content+="# Product: $PRODUCT\n"
                fi
                if [ -n "$symlink" ]; then
                    rules_content+="# Symlink: /dev/$symlink\n"
                fi
                rules_content+="$rule\n\n"
                
                ((count++))
                print_success "已为 $dev 生成规则"
                if [ -n "$symlink" ]; then
                    print_info "符号链接: /dev/$symlink"
                fi
            fi
        else
            print_warning "跳过 $dev (无法获取必要属性)"
        fi
    done
    
    if [ $count -gt 0 ]; then
        echo -e "$rules_content" > "$LOCAL_RULES_FILE"
        print_success "共生成 $count 条规则，保存到: $(pwd)/$LOCAL_RULES_FILE"
        
        echo ""
        echo "----------------------------------------------"
        echo "  生成的规则文件内容:"
        echo "----------------------------------------------"
        cat "$LOCAL_RULES_FILE"
        echo "----------------------------------------------"
    else
        print_warning "未生成任何规则"
    fi
    
    return 0
}

#-------------------------------------------------------------------------------
# 安装规则文件到系统
#-------------------------------------------------------------------------------
install_rules() {
    # 如果没有设置规则文件名，列出当前目录的 .rules 文件让用户选择
    if [ -z "$LOCAL_RULES_FILE" ]; then
        local rules_files=$(ls *.rules 2>/dev/null)
        
        if [ -z "$rules_files" ]; then
            print_error "当前目录没有 .rules 文件，请先生成规则"
            return 1
        fi
        
        echo ""
        echo "当前目录的规则文件:"
        local i=1
        for f in $rules_files; do
            echo "  $i) $f"
            ((i++))
        done
        echo ""
        read -p "请输入要安装的规则文件名: " LOCAL_RULES_FILE
    fi
    
    if [ ! -f "$LOCAL_RULES_FILE" ]; then
        print_error "规则文件 $LOCAL_RULES_FILE 不存在，请先生成规则"
        return 1
    fi
    
    echo ""
    echo "=============================================="
    echo "        安装udev规则文件"
    echo "=============================================="
    echo ""
    
    print_info "将规则文件拷贝到 $RULES_DIR/"
    
    if sudo cp "$LOCAL_RULES_FILE" "$RULES_DIR/"; then
        print_success "规则文件已拷贝到 $RULES_DIR/$LOCAL_RULES_FILE"
        
        print_info "重新加载udev规则..."
        sudo udevadm control --reload-rules
        
        print_info "触发udev规则..."
        sudo udevadm trigger
        
        print_success "udev规则已安装并生效！"
        
        echo ""
        print_info "您可能需要重新插拔USB设备使新规则生效"
    else
        print_error "拷贝规则文件失败，请检查权限"
        return 1
    fi
    
    return 0
}

#-------------------------------------------------------------------------------
# 显示帮助信息
#-------------------------------------------------------------------------------
show_help() {
    echo ""
    echo "USB设备udev规则生成脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -l, --list        列出所有ttyUSB设备"
    echo "  -i, --info DEV    查看指定设备的详细信息"
    echo "                    例如: $0 -i /dev/ttyUSB0"
    echo "  -g, --generate    交互式生成udev规则"
    echo "  -a, --auto        为所有设备生成规则"
    echo "  -s, --install     安装规则文件到系统"
    echo "  -h, --help        显示此帮助信息"
    echo ""
    echo "功能说明:"
    echo "  - 规则文件名：可自定义，如 99-my-serial.rules"
    echo "  - 符号链接名：可自定义，如 serial_port_1 (将创建 /dev/serial_port_1)"
    echo ""
    echo "示例:"
    echo "  $0 -l                    # 列出所有USB串口设备"
    echo "  $0 -i /dev/ttyUSB0       # 查看ttyUSB0的详细信息"
    echo "  $0 -g                    # 交互式生成规则"
    echo "  $0 -a                    # 为所有设备生成规则"
    echo "  $0 -a -s                 # 生成并安装规则"
    echo "  $0 -s                    # 安装已有的规则文件"
    echo ""
}

#-------------------------------------------------------------------------------
# 主菜单
#-------------------------------------------------------------------------------
main_menu() {
    while true; do
        echo ""
        echo "=============================================="
        echo "      USB设备udev规则生成工具"
        echo "=============================================="
        echo ""
        echo "  1) 列出所有ttyUSB设备"
        echo "  2) 查看指定设备详细信息"
        echo "  3) 交互式生成udev规则"
        echo "  4) 自动为所有设备生成规则"
        echo "  5) 安装规则文件到系统"
        echo "  6) 退出"
        echo ""
        read -p "请选择操作 [1-6]: " choice
        
        case $choice in
            1)
                list_usb_devices
                ;;
            2)
                list_usb_devices
                if [ $? -eq 0 ]; then
                    read -p "请输入设备路径 (例如 /dev/ttyUSB0): " dev
                    get_device_info "$dev"
                fi
                ;;
            3)
                interactive_generate_rules
                ;;
            4)
                auto_generate_rules
                ;;
            5)
                install_rules
                ;;
            6)
                echo ""
                print_info "再见！"
                exit 0
                ;;
            *)
                print_error "无效的选择"
                ;;
        esac
    done
}

#-------------------------------------------------------------------------------
# 主程序入口
#-------------------------------------------------------------------------------
main() {
    # 检查是否有参数
    if [ $# -eq 0 ]; then
        main_menu
        exit 0
    fi
    
    local do_install=false
    
    # 解析命令行参数
    while [ $# -gt 0 ]; do
        case "$1" in
            -l|--list)
                list_usb_devices
                shift
                ;;
            -i|--info)
                if [ -n "$2" ]; then
                    get_device_info "$2"
                    show_device_attrs "$2"
                    shift 2
                else
                    print_error "请指定设备路径"
                    exit 1
                fi
                ;;
            -g|--generate)
                interactive_generate_rules
                shift
                ;;
            -a|--auto)
                auto_generate_rules
                shift
                ;;
            -s|--install)
                do_install=true
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                print_error "未知选项: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # 如果指定了安装选项，执行安装
    if [ "$do_install" = true ]; then
        install_rules
    fi
}

# 运行主程序
main "$@"
