#!/usr/bin/env python3
"""
检查速度和加速度设置
"""

import serial
import time


def checksum(data):
    return (~sum(data)) & 0xFF


def read_reg(ser, servo_id, address, length):
    params = [address, length]
    data = [servo_id, 4, 0x02] + params
    chk = checksum(data)
    packet = bytes([0xFF, 0xFF] + data + [chk])
    
    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.1)
    resp = ser.read(50)
    
    if resp and len(resp) >= 6:
        resp_len = resp[3]
        if resp_len > 2 and len(resp) >= 5 + (resp_len - 2):
            return resp[5:5 + (resp_len - 2)]
    return None


def write_reg(ser, servo_id, address, values):
    if isinstance(values, int):
        values = [values]
    
    params = [address] + values
    length = len(params) + 2
    data = [servo_id, length, 0x03] + params
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
    print("速度和加速度检查")
    print("="*60)
    
    try:
        ser = serial.Serial(port, 1000000, timeout=0.5)
        time.sleep(0.5)
        print(f"✓ 连接到 {port}\n")
        
        servo_id = 1
        
        # 启用扭矩
        print("启用扭矩...")
        write_reg(ser, servo_id, 40, [1])
        
        # 读取速度 (地址46)
        print("\n读取速度 (地址46)...")
        data = read_reg(ser, servo_id, 46, 2)
        if data and len(data) >= 2:
            speed = data[0] + (data[1] << 8)
            print(f"  速度: {speed}")
        else:
            print("  读取失败")
        
        # 读取加速度 (地址44)
        print("\n读取加速度 (地址44)...")
        data = read_reg(ser, servo_id, 44, 1)
        if data:
            accel = data[0]
            print(f"  加速度: {accel}")
        else:
            print("  读取失败")
        
        # 设置速度=500
        print("\n设置速度=500...")
        speed = 500
        spd_low = speed & 0xFF
        spd_high = (speed >> 8) & 0xFF
        write_reg(ser, servo_id, 46, [spd_low, spd_high])
        
        # 设置加速度=50
        print("设置加速度=50...")
        write_reg(ser, servo_id, 44, [50])
        
        # 再次读取
        print("\n再次读取速度...")
        data = read_reg(ser, servo_id, 46, 2)
        if data and len(data) >= 2:
            speed = data[0] + (data[1] << 8)
            print(f"  速度: {speed}")
        
        print("\n再次读取加速度...")
        data = read_reg(ser, servo_id, 44, 1)
        if data:
            print(f"  加速度: {data[0]}")
        
        # 读取当前位置
        print("\n当前位置...")
        data = read_reg(ser, servo_id, 56, 2)
        if data and len(data) >= 2:
            pos = data[0] + (data[1] << 8)
            print(f"  位置: {pos}")
        
        # 写入目标位置
        print("\n写入目标位置=1500...")
        pos = 1500
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        write_reg(ser, servo_id, 42, [pos_low, pos_high])
        
        print("等待2秒...")
        time.sleep(2)
        
        # 读取当前位置
        print("\n当前位置...")
        data = read_reg(ser, servo_id, 56, 2)
        if data and len(data) >= 2:
            pos = data[0] + (data[1] << 8)
            print(f"  位置: {pos}")
        
        # 写入目标位置=2500
        print("\n写入目标位置=2500...")
        pos = 2500
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        write_reg(ser, servo_id, 42, [pos_low, pos_high])
        
        print("等待2秒...")
        time.sleep(2)
        
        # 读取当前位置
        print("\n当前位置...")
        data = read_reg(ser, servo_id, 56, 2)
        if data and len(data) >= 2:
            pos = data[0] + (data[1] << 8)
            print(f"  位置: {pos}")
        
        ser.close()
        print("\n测试完成!")
        
    except Exception as e:
        print(f"错误: {e}")


if __name__ == "__main__":
    main()
    input("\n按Enter退出...")