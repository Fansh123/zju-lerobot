#!/usr/bin/env python3
"""
Waveshare SO-ARM101 串口调试工具
测试实际通信，找出问题所在
"""

import serial
import time
import struct

def test_feetech_protocol(port, baudrate=1000000):
    """测试 Feetech STS3215 协议"""
    print(f"\n测试 {port} @ {baudrate} baud")
    print("="*60)
    
    try:
        ser = serial.Serial(
            port, 
            baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )
        time.sleep(0.5)
        print("✓ 串口打开成功")
        
        # 清空缓冲区
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Feetech STS3215 协议测试
        
        # 测试1: 读取舵机1位置
        print("\n测试1: 读取舵机1位置")
        # 指令: FF FF ID LEN INST PARAM... CHECK
        packet1 = bytes([0xFF, 0xFF, 0x01, 0x04, 0x2A, 0x38, 0x00])
        # 计算校验和: ID ^ LEN ^ INST ^ ADDR_H ^ ADDR_L
        checksum = 0x01 ^ 0x04 ^ 0x2A ^ 0x38 ^ 0x00
        packet1 = bytes([0xFF, 0xFF, 0x01, 0x04, 0x2A, 0x38, 0x00, checksum & 0xFE])
        
        print(f"  发送: {packet1.hex()}")
        ser.write(packet1)
        time.sleep(0.2)
        
        response = ser.read(10)
        print(f"  收到: {response.hex() if response else '无响应'}")
        
        if response and len(response) >= 6:
            print(f"  解析: ID={response[2]}, 位置低={response[5]}, 位置高={response[6]}")
            if len(response) >= 8:
                pos = response[5] + (response[6] << 8)
                print(f"  位置值: {pos} (范围 0-4095)")
        
        # 测试2: 启用舵机1扭矩
        print("\n测试2: 启用舵机1扭矩")
        # 写一个字节到地址 40 (扭矩使能)
        packet2 = bytes([0xFF, 0xFF, 0x01, 0x05, 0x28, 0x28, 0x01, 0x01])
        checksum2 = 0x01 ^ 0x05 ^ 0x28 ^ 0x28 ^ 0x01
        packet2 = bytes([0xFF, 0xFF, 0x01, 0x05, 0x28, 0x28, 0x01, checksum2 & 0xFE])
        
        print(f"  发送: {packet2.hex()}")
        ser.write(packet2)
        time.sleep(0.1)
        
        response2 = ser.read(10)
        print(f"  收到: {response2.hex() if response2 else '无响应'}")
        
        # 测试3: 设置舵机1位置到中心
        print("\n测试3: 设置舵机1位置到中心 (2048)")
        # 写2字节位置 + 2字节速度 到地址 42
        pos = 2048
        speed = 500
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        speed_low = speed & 0xFF
        speed_high = (speed >> 8) & 0xFF
        
        packet3 = bytes([0xFF, 0xFF, 0x01, 0x07, 0x28, 0x2A, pos_low, pos_high, speed_low, speed_high])
        checksum3 = 0x01 ^ 0x07 ^ 0x28 ^ 0x2A ^ pos_low ^ pos_high ^ speed_low ^ speed_high
        packet3 = packet3 + bytes([checksum3 & 0xFE])
        
        print(f"  发送: {packet3.hex()}")
        ser.write(packet3)
        time.sleep(0.5)
        
        response3 = ser.read(10)
        print(f"  收到: {response3.hex() if response3 else '无响应'}")
        
        print("\n如果机械臂没有动，请检查:")
        print("  1. 电源是否连接并打开")
        print("  2. 舵机ID是否正确 (通常是1-6)")
        print("  3. 控制板跳线设置是否正确")
        
        ser.close()
        
    except Exception as e:
        print(f"✗ 错误: {e}")


def test_waveshare_protocol(port, baudrate=115200):
    """测试 Waveshare 可能的协议"""
    print(f"\n测试 Waveshare 协议 {port} @ {baudrate} baud")
    print("="*60)
    
    for baud in [115200, 1000000, 57600, 9600]:
        try:
            ser = serial.Serial(port, baud, timeout=0.5)
            time.sleep(0.3)
            print(f"\n波特率 {baud}:")
            
            # 尝试发送读取指令
            test_commands = [
                ("标准读", bytes([0x55, 0x55, 0x01, 0x03, 0x00])),
                ("舵机读", bytes([0xFF, 0xFF, 0x01, 0x04, 0x24, 0x38, 0x00, 0x00])),
                ("Ping", bytes([0x55, 0x55, 0x00, 0x01, 0x00])),
            ]
            
            for name, cmd in test_commands:
                ser.reset_input_buffer()
                ser.write(cmd)
                time.sleep(0.2)
                resp = ser.read(20)
                if resp:
                    print(f"  {name}: {resp.hex()}")
            
            ser.close()
            
        except Exception as e:
            print(f"  波特率 {baud} 失败: {e}")


def test_all_baudrates(port):
    """测试所有可能的波特率"""
    print(f"\n扫描 {port} 所有波特率")
    print("="*60)
    
    baudrates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 1000000, 2000000]
    
    for baud in baudrates:
        try:
            ser = serial.Serial(port, baud, timeout=0.2)
            time.sleep(0.1)
            
            # 发送一个简单的指令
            ser.write(bytes([0xFF, 0xFF, 0x01, 0x02, 0x02, 0x00, 0x00]))
            time.sleep(0.1)
            
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(f"✓ {baud:>10} baud: 收到 {len(data)} 字节 - {data.hex()}")
            else:
                print(f"  {baud:>10} baud: 无响应")
            
            ser.close()
            
        except Exception as e:
            print(f"✗ {baud:>10} baud: {e}")


def main():
    import sys
    
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("Waveshare SO-ARM101 通信调试")
    print("="*60)
    
    # 1. 测试 Feetech 协议 (1 Mbps)
    test_feetech_protocol(port, 1000000)
    
    # 2. 扫描所有波特率
    test_all_baudrates(port)
    
    # 3. 测试 Waveshare 可能的协议
    test_waveshare_protocol(port, 115200)


if __name__ == "__main__":
    main()
    input("\n按 Enter 退出...")