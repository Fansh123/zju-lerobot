#!/usr/bin/env python3
"""
串口调试工具
测试不同协议，找到正确的通信方式
"""

import serial
import time
import struct

def test_port(port_name, baudrate=115200):
    """测试串口通信"""
    print(f"\n测试端口: {port_name} @ {baudrate} baud")
    print("="*60)
    
    try:
        ser = serial.Serial(port_name, baudrate, timeout=1)
        time.sleep(0.5)
        print(f"✓ 串口打开成功")
        
        # 清空缓冲区
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # 测试各种协议
        tests = [
            ("标准STS3215读取", bytes([0xFF, 0xFF, 0x01, 0x02, 0x02, 0x00, 0x05])),
            ("舵机1位置写", bytes([0xFF, 0xFF, 0x01, 0x03, 0x00, 0x00, 0x04])),
            ("舵机2位置写", bytes([0xFF, 0xFF, 0x02, 0x03, 0x00, 0x00, 0x05])),
            ("广播读取", bytes([0xFF, 0xFF, 0xFE, 0x02, 0x02, 0x00, 0x02])),
            ("Waveshare指令1", bytes([0x55, 0x55, 0x01, 0x03, 0x00, 0x00])),
            ("简单ping", bytes([0xAA])),
            ("舵机1移动500", bytes([0xFF, 0xFF, 0x01, 0x05, 0x03, 0x00, 0xF4, 0x01, 0xF9])),
        ]
        
        for name, cmd in tests:
            print(f"\n测试: {name}")
            print(f"  发送: {cmd.hex()}")
            
            try:
                ser.write(cmd)
                time.sleep(0.3)
                
                # 尝试读取响应
                response = ser.read(20)
                
                if response:
                    print(f"  收到: {response.hex()}")
                    if len(response) > 0:
                        print(f"  字节数: {len(response)}")
                else:
                    print(f"  无响应")
                    
            except Exception as e:
                print(f"  错误: {e}")
            
            time.sleep(0.2)
        
        # 尝试读取原始数据流
        print("\n\n尝试读取串口数据流 (5秒)...")
        ser.reset_input_buffer()
        start = time.time()
        while time.time() - start < 5:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                if data:
                    print(f"收到数据: {data.hex()}")
            time.sleep(0.1)
        
        ser.close()
        print("\n串口已关闭")
        return True
        
    except Exception as e:
        print(f"✗ 错误: {e}")
        return False


def main():
    import sys
    
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    print("="*60)
    print("SO-ARM101 串口调试工具")
    print("="*60)
    
    test_port(port, baud)

if __name__ == "__main__":
    main()