#!/usr/bin/env python3
"""
Linux端编码器数据接收服务器

功能:
1. TCP Socket服务器，接收ESP32发送的编码器数据
2. 解析JSON数据并保存到文件/数据库
3. 可选: 实时显示数据

使用方法:
    python3 receiver.py [--port 8888] [--output data.json]
"""

import socket
import json
import argparse
import logging
from datetime import datetime
from pathlib import Path
import threading
import signal
import sys

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class EncoderDataReceiver:
    """编码器数据接收服务器"""
    
    def __init__(self, host='0.0.0.0', port=8888, output_dir='./data'):
        self.host = host
        self.port = port
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.server_socket = None
        self.running = False
        self.data_count = 0
        
    def start(self):
        """启动服务器"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.server_socket.settimeout(1.0)  # 允许定期检查running标志
        
        self.running = True
        logger.info(f"服务器已启动: {self.host}:{self.port}")
        logger.info(f"数据保存目录: {self.output_dir.absolute()}")
        logger.info("等待ESP32连接...")
        
        while self.running:
            try:
                client_socket, client_addr = self.server_socket.accept()
                logger.info(f"新连接: {client_addr}")
                
                # 在新线程中处理客户端
                thread = threading.Thread(
                    target=self.handle_client,
                    args=(client_socket, client_addr)
                )
                thread.daemon = True
                thread.start()
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    logger.error(f"接受连接时出错: {e}")
                    
    def handle_client(self, client_socket, client_addr):
        """处理客户端连接"""
        try:
            # 接收数据
            data = b''
            while True:
                chunk = client_socket.recv(4096)
                if not chunk:
                    break
                data += chunk
                # 检查是否收到完整JSON
                if data.endswith(b'\n') or data.endswith(b'}'):
                    break
            
            if data:
                self.process_data(data.decode('utf-8'), client_addr)
                # 发送确认响应
                client_socket.send(b'OK\n')
                
        except Exception as e:
            logger.error(f"处理客户端数据时出错: {e}")
        finally:
            client_socket.close()
            
    def process_data(self, raw_data, client_addr):
        """处理接收到的数据"""
        try:
            # 解析JSON
            data = json.loads(raw_data.strip())
            
            device_id = data.get('device_id', 'unknown')
            sample_count = data.get('sample_count', 0)
            samples = data.get('samples', [])
            
            logger.info(f"收到数据 - 设备: {device_id}, 样本数: {sample_count}")
            
            # 打印最新一条数据
            if samples:
                latest = samples[-1]
                values = latest.get('values', [])
                logger.info(f"最新数据: 时间戳={latest.get('ts')}ms")
                logger.info(f"  编码器值: {[f'{v:.2f}°' for v in values[:5]]}... (共{len(values)}个)")
            
            # 保存到文件
            self.save_data(data, device_id)
            self.data_count += 1
            
        except json.JSONDecodeError as e:
            logger.error(f"JSON解析错误: {e}")
            logger.debug(f"原始数据: {raw_data[:200]}...")
            
    def save_data(self, data, device_id):
        """保存数据到文件"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = self.output_dir / f"encoder_data_{device_id.replace(':', '')}_{timestamp}.json"
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
            
        logger.info(f"数据已保存: {filename}")
        
    def stop(self):
        """停止服务器"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
        logger.info(f"服务器已停止, 共接收 {self.data_count} 次数据")


class MQTTReceiver:
    """
    MQTT方式接收数据 (可选方案)
    需要安装: pip install paho-mqtt
    """
    
    def __init__(self, broker='localhost', port=1883, topic='encoder/data'):
        self.broker = broker
        self.port = port
        self.topic = topic
        self.client = None
        
    def start(self):
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            logger.error("请安装paho-mqtt: pip install paho-mqtt")
            return
            
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        self.client.connect(self.broker, self.port, 60)
        logger.info(f"MQTT客户端已连接: {self.broker}:{self.port}")
        self.client.loop_forever()
        
    def on_connect(self, client, userdata, flags, rc):
        logger.info(f"MQTT连接成功, 订阅主题: {self.topic}")
        client.subscribe(self.topic)
        
    def on_message(self, client, userdata, msg):
        logger.info(f"收到MQTT消息: {msg.topic}")
        try:
            data = json.loads(msg.payload.decode())
            logger.info(f"数据: {json.dumps(data, indent=2)[:500]}...")
        except:
            logger.error("消息解析失败")


def signal_handler(sig, frame):
    """处理Ctrl+C"""
    logger.info("\n正在关闭服务器...")
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='编码器数据接收服务器')
    parser.add_argument('--host', default='0.0.0.0', help='监听地址')
    parser.add_argument('--port', type=int, default=8888, help='监听端口')
    parser.add_argument('--output', default='./data', help='数据保存目录')
    parser.add_argument('--mqtt', action='store_true', help='使用MQTT模式')
    parser.add_argument('--mqtt-broker', default='localhost', help='MQTT Broker地址')
    
    args = parser.parse_args()
    
    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print("""
╔═══════════════════════════════════════════════════════╗
║        编码器数据接收服务器 (Linux端)                  ║
║                                                       ║
║  配置ESP32端的服务器IP为本机IP地址                     ║
║  按Ctrl+C停止服务器                                   ║
╚═══════════════════════════════════════════════════════╝
    """)
    
    if args.mqtt:
        receiver = MQTTReceiver(broker=args.mqtt_broker)
    else:
        receiver = EncoderDataReceiver(
            host=args.host,
            port=args.port,
            output_dir=args.output
        )
    
    try:
        receiver.start()
    except KeyboardInterrupt:
        receiver.stop()


if __name__ == '__main__':
    main()
