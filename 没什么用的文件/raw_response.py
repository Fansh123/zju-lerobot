#!/usr/bin/env python3
"""
查看原始响应数据
"""

import serial
import time


def checksum(data):
    return (~sum(data)) & 0xFF


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("原始响应数据分析")
    print("="*60)
    
    try:
        ser = serial.Serial(port, 1000000, timeout=0.3)
        time.sleep(0.3)
        print(f"✓ 连接到 {port}\n")
        
        # 测试读取舵机1的位置
        print("读取舵机1位置 (地址56, 2字节):")
        
        # 构建读指令
        params = [56, 2]  # 地址56, 读取2字节
        data = [1, 4, 0x2A] + params
        chk = checksum(data)
        packet = bytes([0xFF, 0xFF] + data + [chk])
        
        print(f"发送: {packet.hex()}")
        
        ser.reset_input_buffer()
        ser.write(packet)
        time.sleep(0.2)
        
        response = ser.read(50)
        print(f"收到: {response.hex()}")
        print(f"长度: {len(response)}")
        
        if len(response) >= 8:
            print("\n解析:")
            print(f"  Header: {response[0]:02x} {response[1]:02x}")
            print(f"  ID: {response[2]}")
            print(f"  Length: {response[3]}")
            print(f"  Error: {response[4]}")
            if response[3] >= 4:
                print(f"  Param1: {response[5]}")
                print(f"  Param2: {response[6]}")
                pos = response[5] + (response[6] << 8)
                print(f"  位置值: {pos}")
            print(f"  Checksum: {response[-1] if len(response) > 5 else 'N/A'}")
        
        # 测试PING
        print("\n\nPING舵机1:")
        data = [1, 2, 0x01]
        chk = checksum(data)
        packet = bytes([0xFF, 0xFF] + data + [chk])
        print(f"发送: {packet.hex()}")
        
        ser.reset_input_buffer()
        ser.write(packet)
        time.sleep(0.2)
        
        response = ser.read(20)
        print(f"收到: {response.hex()}")
        print(f"长度: {len(response)}")
        
        ser.close()
        
    except Exception as e:
        print(f"错误: {e}")


if __name__ == "__main__":
    main()
    input("\n按Enter退出...")