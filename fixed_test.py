#!/usr/bin/env python3
"""
修复校验和的通信测试
Feetech SCS/STS 协议校验和 = ~(ID + Length + Instruction + Parameters) & 0xFF
之前用的是 XOR，完全错误！
"""

import serial
import time


def feetech_checksum(data):
    """计算 Feetech SCS 协议校验和"""
    # data = [ID, Length, Instruction, Param1, Param2, ...]
    total = sum(data) & 0xFF
    return (~total) & 0xFF


def build_packet(servo_id, instruction, parameters):
    """构建 Feetech 协议数据包"""
    length = len(parameters) + 2  # instruction + parameters + checksum
    data = [servo_id, length, instruction] + parameters
    checksum = feetech_checksum(data)
    return bytes([0xFF, 0xFF] + data + [checksum])


def test_fixed_commands(port='COM18'):
    """使用正确校验和测试通信"""
    
    print("="*60)
    print("修复校验和后的通信测试")
    print("="*60)
    
    for baudrate in [1000000, 115200]:
        print(f"\n{'='*60}")
        print(f"波特率: {baudrate}")
        print("="*60)
        
        try:
            ser = serial.Serial(
                port, baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            time.sleep(0.5)
            print(f"✓ 串口打开成功")
            
            # 测试1: PING 舵机1
            print("\n--- 测试1: PING 舵机1 ---")
            packet = build_packet(0x01, 0x01, [])  # INST_PING = 0x01
            print(f"发送: {packet.hex()}")
            ser.reset_input_buffer()
            ser.write(packet)
            time.sleep(0.3)
            response = ser.read(20)
            print(f"收到: {response.hex() if response else '(无)'}")
            if response:
                print(f"长度: {len(response)}")
            
            # 测试2: PING 舵机2
            print("\n--- 测试2: PING 舵机2 ---")
            packet = build_packet(0x02, 0x01, [])
            print(f"发送: {packet.hex()}")
            ser.reset_input_buffer()
            ser.write(packet)
            time.sleep(0.3)
            response = ser.read(20)
            print(f"收到: {response.hex() if response else '(无)'}")
            
            # 测试3: 读取舵机1位置 (地址56, 2字节)
            print("\n--- 测试3: 读取舵机1位置 ---")
            # INST_READ = 0x2A, 地址=0x38(56), 长度=0x02
            packet = build_packet(0x01, 0x2A, [0x38, 0x02])
            print(f"发送: {packet.hex()}")
            ser.reset_input_buffer()
            ser.write(packet)
            time.sleep(0.3)
            response = ser.read(20)
            print(f"收到: {response.hex() if response else '(无)'}")
            if response and len(response) >= 8:
                pos = response[5] + (response[6] << 8)
                print(f"位置值: {pos} (0-4095, 中心=2048)")
            
            # 测试4: 启用舵机1扭矩
            print("\n--- 测试4: 启用舵机1扭矩 ---")
            # INST_WRITE = 0x28, 地址=0x28(40), 值=0x01
            packet = build_packet(0x01, 0x28, [0x28, 0x01])
            print(f"发送: {packet.hex()}")
            ser.reset_input_buffer()
            ser.write(packet)
            time.sleep(0.3)
            response = ser.read(20)
            print(f"收到: {response.hex() if response else '(无)'}")
            
            # 测试5: 设置舵机1位置到中心 (2048)
            print("\n--- 测试5: 设置舵机1位置到2048 ---")
            pos = 2048
            speed = 200
            pos_low = pos & 0xFF
            pos_high = (pos >> 8) & 0xFF
            speed_low = speed & 0xFF
            speed_high = (speed >> 8) & 0xFF
            # INST_WRITE = 0x28, 地址=0x2A(42)
            packet = build_packet(0x01, 0x28, [0x2A, pos_low, pos_high, speed_low, speed_high])
            print(f"发送: {packet.hex()}")
            ser.reset_input_buffer()
            ser.write(packet)
            time.sleep(1.0)
            response = ser.read(20)
            print(f"收到: {response.hex() if response else '(无)'}")
            
            # 测试6: 扫描所有舵机ID
            print("\n--- 测试6: 扫描舵机ID (1-10) ---")
            for servo_id in range(1, 11):
                packet = build_packet(servo_id, 0x01, [])  # PING
                ser.reset_input_buffer()
                ser.write(packet)
                time.sleep(0.2)
                response = ser.read(10)
                if response and len(response) >= 4:
                    resp_id = response[2]
                    if resp_id == servo_id:
                        print(f"  ✓ 舵机 ID={servo_id} 响应! 数据: {response.hex()}")
                    else:
                        print(f"  ? ID={servo_id}: 收到数据但ID不匹配: {response.hex()}")
                else:
                    print(f"  ✗ ID={servo_id}: 无响应")
            
            ser.close()
            
        except Exception as e:
            print(f"错误: {e}")


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    test_fixed_commands(port)
    
    print("\n" + "="*60)
    print("测试完成")
    print("="*60)


if __name__ == "__main__":
    main()
    input("\n按Enter退出...")