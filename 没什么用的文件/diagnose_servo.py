#!/usr/bin/env python3
"""
舵机状态诊断
读取舵机的实际状态
"""

import serial
import time


def checksum(data):
    return (~sum(data)) & 0xFF


def read_register(ser, servo_id, address, length):
    """读取寄存器"""
    params = [address, length]
    data = [servo_id, 4, 0x2A] + params
    chk = checksum(data)
    packet = bytes([0xFF, 0xFF] + data + [chk])
    
    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.1)
    return ser.read(20)


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("舵机状态诊断")
    print("="*60)
    
    try:
        ser = serial.Serial(port, 1000000, timeout=0.2)
        time.sleep(0.3)
        print(f"✓ 连接到 {port}\n")
        
        for servo_id in range(1, 7):
            print(f"--- 舵机 ID={servo_id} ---")
            
            # 读取扭矩使能状态 (地址40)
            resp = read_register(ser, servo_id, 40, 1)
            if resp and len(resp) >= 6:
                torque = resp[5] if len(resp) > 5 else 0
                print(f"  扭矩使能: {torque} ({'启用' if torque else '禁用'})")
            else:
                print(f"  扭矩使能: 读取失败")
            
            # 读取当前位置 (地址56, 2字节)
            resp = read_register(ser, servo_id, 56, 2)
            if resp and len(resp) >= 8:
                pos = resp[5] + (resp[6] << 8)
                print(f"  当前位置: {pos}")
            else:
                print(f"  当前位置: 读取失败")
            
            # 读取电压 (地址62)
            resp = read_register(ser, servo_id, 62, 1)
            if resp and len(resp) >= 6:
                voltage = resp[5] / 10.0  # 单位是0.1V
                print(f"  电压: {voltage}V")
            else:
                print(f"  电压: 读取失败")
            
            # 读取温度 (地址63)
            resp = read_register(ser, servo_id, 63, 1)
            if resp and len(resp) >= 6:
                temp = resp[5]
                print(f"  温度: {temp}°C")
            else:
                print(f"  温度: 读取失败")
            
            print()
        
        ser.close()
        
    except Exception as e:
        print(f"错误: {e}")


if __name__ == "__main__":
    main()
    input("\n按Enter退出...")