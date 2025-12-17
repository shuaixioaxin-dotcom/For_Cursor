# Luckfox Pico SDK（中文说明）

![luckfox](https://github.com/LuckfoxTECH/luckfox-pico/assets/144299491/cec5c4a5-22b9-4a9a-abb1-704b11651e88)

- 本 SDK 基于 Rockchip 官方 SDK 修改
- 为 Luckfox Pico 系列开发板提供定制化开发套件
- 目标是给开发者提供更好的编译/开发体验

## SDK 更新日志

- 当前版本：V1.4

1. 更新 U-Boot：支持 RV1106 在 SPI NAND 与 eMMC 场景下的快速启动（fast boot）。
2. 优化 U-Boot 与 SD 卡的兼容性，降低 SD 卡识别失败概率。
3. 内核更新至 5.10.160，并提升 RV1106G3 的 NPU 频率。
4. 更新 Buildroot 镜像源，提升软件包下载稳定性。
5. 新增对自定义文件系统的支持。
6. 修复部分问题。

## SDK 使用说明

- 推荐操作系统：Ubuntu 22.04

### 安装依赖

```shell
sudo apt-get install -y git ssh make gcc gcc-multilib g++-multilib module-assistant expect g++ gawk texinfo libssl-dev bison flex fakeroot cmake unzip gperf autoconf device-tree-compiler libncurses5-dev pkg-config bc python-is-python3 passwd openssl openssh-server openssh-client vim file cpio rsync
```

### 获取 SDK

```shell
git clone https://github.com/LuckfoxTECH/luckfox-pico.git
```

### 设置交叉编译工具链环境变量

交叉编译工具链需要先配置环境变量：

```shell
cd {SDK_PATH}/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/
source env_install_toolchain.sh
```

### 仓库地址

- GitHub

```shell
git clone https://github.com/LuckfoxTECH/luckfox-pico.git
```

- Gitee

```shell
git clone https://gitee.com/LuckfoxTECH/luckfox-pico.git
```

## build.sh 使用说明

`build.sh` 用于自动化编译流程，大部分编译操作都可以通过它一键完成。

### build.sh 选项

```shell
Usage: build.sh [OPTIONS]
Available options:
lunch              -Select Board Configure
env                -build env
meta               -build meta (optional)
uboot              -build uboot
kernel             -build kernel
rootfs             -build rootfs
driver             -build kernel's drivers
sysdrv             -build uboot, kernel, rootfs
media              -build rockchip media libraries
app                -build app
recovery           -build recovery
tool               -build tool
updateimg          -build update image
unpackimg          -unpack update image
factory            -build factory image
all                -build uboot, kernel, rootfs, recovery image
allsave            -build all & firmware & save

clean              -clean all
clean uboot        -clean uboot
clean kernel       -clean kernel
clean driver       -clean driver
clean rootfs       -clean rootfs
clean sysdrv       -clean uboot/kernel/rootfs
clean media        -clean rockchip media libraries
clean app          -clean app
clean recovery     -clean recovery

firmware           -pack all the image we need to boot up system
ota                -pack update_ota.tar
save               -save images, patches, commands used to debug
check              -check the environment of building
info               -see the current board building information

buildrootconfig    -config buildroot and save defconfig"
kernelconfig       -config kernel and save defconfig"
```

### 选择参考板级配置（lunch）

```shell
./build.sh lunch
```

执行后会依次让你选择：

- Luckfox Pico 硬件型号（输入序号；直接回车默认选 `[0]`）
- 启动介质（例如 SD_CARD / SPI_NAND 等）
- 根文件系统类型（例如 Buildroot 等）

如需使用旧式配置方式或自定义板级支持文件，可在硬件型号选择中选 `custom`，然后在列出的 BoardConfig 文件中选择对应条目完成配置。

### 设置 Buildroot 默认 WiFi 配置

进入板级配置目录：

```shell
cd {SDK_PATH}/project/cfg/BoardConfig_IPC/
```

打开对应的板级配置文件，修改/导出以下参数（SSID/密码替换为你自己的）：

```shell
export LF_WIFI_SSID="Your wifi ssid"
export LF_WIFI_PSK="Your wifi password"
```

## 一键自动编译

```shell
./build.sh lunch   # 选择参考板级配置
./build.sh         # 一键自动编译
```

## 分项编译

### 编译 U-Boot

```shell
./build.sh clean uboot
./build.sh uboot
```

产物路径：

```text
output/image/MiniLoaderAll.bin
output/image/uboot.img
```

### 编译 Kernel

```shell
./build.sh clean kernel
./build.sh kernel
```

产物路径：

```text
output/image/boot.img
```

### 编译 Rootfs

```shell
./build.sh clean rootfs
./build.sh rootfs
```

注意：编译完成后，通常需要执行 `./build.sh firmware` 重新打包固件镜像。

### 编译 Media

```shell
./build.sh clean media
./build.sh media
```

产物路径：

```text
output/out/media_out
```

注意：编译完成后，通常需要执行 `./build.sh firmware` 重新打包固件镜像。

### 编译参考应用（App）

```shell
./build.sh clean app
./build.sh app
```

- 注意 1：`app` 依赖 `media`。
- 注意 2：编译完成后，通常需要执行 `./build.sh firmware` 重新打包固件镜像。

## 编译镜像说明（Firmware Packaging）

当你完成 U-Boot/Kernel/Rootfs（以及可选的 media/app）编译后，通过 `firmware` 目标把系统启动所需镜像打包到 `output/image`：

```shell
./build.sh firmware
```

产物路径：

```text
output/image
```

## 配置入口

### Kernel 配置

```shell
./build.sh kernelconfig
```

打开内核的 menuconfig 界面。

### Buildroot 配置

```shell
./build.sh buildrootconfig
```

打开 Buildroot 的 menuconfig 界面。

- 注意：仅在 rootfs 选择 Buildroot 时适用。

## 注意事项

在 Windows 下拷贝源码包时，Linux 下的可执行权限可能丢失、软链接可能失效，从而导致无法编译或无法使用。
因此请尽量避免在 Windows 环境中“复制/解压/搬运”源码包后再到 Linux 编译。

