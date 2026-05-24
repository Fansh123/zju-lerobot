#!/usr/bin/env python3
"""
使用正确的指令码测试舵机
指令码: PING=0x01, READ=0x02, WRITE=0x03
"""

import serial
import time


def checksum(data):
    return (~sum(data)) & 0xFF


def ping(ser, servo_id):
    """PING指令 - 检查舵机状态"""
    data = [servo_id, 2, 0x01]  # ID, Length=2, PING=0x01
    chk = checksum(data)
    packet = bytes([0xFF, 0xFF] + data + [chk])
    
    print(f"  PING: {packet.hex()}")
    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.1)
    resp = ser.read(20)
    print(f"  响应: {resp.hex() if resp else '(无)'}")
    return resp


def read_reg(ser, servo_id, address, length):
    """READ指令 - 读取寄存器"""
    params = [address, length]
    data = [servo_id, 4, 0x02] + params  # ID, Length=4, READ=0x02
    chk = checksum(data)
    packet = bytes([0xFF, 0xFF] + data + [chk])
    
    print(f"  READ addr={address}(0x{address:02X}) len={length}: {packet.hex()}")
    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.1)
    resp = ser.read(50)
    print(f"  响应: {resp.hex() if resp else '(无)'}")
    
    if resp and len(resp) >= 6:
        resp_len = resp[3]
        error = resp[4]
        if resp_len > 2 and len(resp) >= 5 + (resp_len - 2):
            data = resp[5:5 + (resp_len - 2)]
            print(f"  数据: {data.hex()}")
            return data
    return None


def write_reg(ser, servo_id, address, values):
    """WRITE指令 - 写入寄存器"""
    if isinstance(values, int):
        values = [values]
    
    params = [address] + values
    length = len(params) + 2
    data = [servo_id, length, 0x03] + params  # WRITE=0x03
    chk = checksum(data)
    packet = bytes([0xFF, 0xFF] + data + [chk])
    
    print(f"  WRITE addr={address}(0x{address:02X}) data={values}: {packet.hex()}")
    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.1)
    resp = ser.read(20)
    print(f"  响应: {resp.hex() if resp else '(无)'}")
    return resp


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("正确指令码测试")
    print("="*60)
    
    try:
        ser = serial.Serial(port, 1000000, timeout=0.5)
        time.sleep(0.5)
        print(f"✓ 连接到 {port}\n")
        
        servo_id = 1
        
        # ===== 测试1: PING =====
        print("=== 测试1: PING ===")
        ping(ser, servo_id)
        
        # ===== 测试2: 读取扭矩状态 (地址40=0x28) =====
        print("\n=== 测试2: 读取扭矩状态 (地址40) ===")
        data = read_reg(ser, servo_id, 40, 1)
        if data:
            print(f"  扭矩状态: {data[0]} ({'启用' if data[0] else '禁用'})")
        
        # ===== 测试3: 写入扭矩使能=1 =====
        print("\n=== 测试3: 写入扭矩使能=1 ===")
        write_reg(ser, servo_id, 40, [1])
        
        # ===== 测试4: 再次读取扭矩状态 =====
        print("\n=== 测试4: 再次读取扭矩状态 ===")
        data = read_reg(ser, servo_id, 40, 1)
        if data:
            print(f"  扭矩状态: {data[0]} ({'启用' if data[0] else '禁用'})")
        
        # ===== 测试5: 读取当前位置 (地址56=0x38) =====
        print("\n=== 测试5: 读取当前位置 (地址56) ===")
        data = read_reg(ser, servo_id, 56, 2)
        if data and len(data) >= 2:
            pos = data[0] + (data[1] << 8)
            print(f"  当前位置: {pos}")
        
        # ===== 测试6: 读取目标位置 (地址42=0x2A) =====
        print("\n=== 测试6: 读取目标位置 (地址42) ===")
        data = read_reg(ser, servo_id, 42, 2)
        if data and len(data) >= 2:
            goal = data[0] + (data[1] << 8)
            print(f"  目标位置: {goal}")
        
        # ===== 测试7: 写入目标位置 =====
        print("\n=== 测试7: 写入目标位置=1500 ===")
        pos = 1500
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        write_reg(ser, servo_id, 42, [pos_low, pos_high])
        
        print("\n等待2秒...")
        time.sleep(2)
        
        # ===== 测试8: 写入目标位置=2500 =====
        print("\n=== 测试8: 写入目标位置=2500 ===")
        pos = 2500
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        write_reg(ser, servo_id, 42, [pos_low, pos_high])
        
        print("\n等待2秒...")
        time.sleep(2)
        
        # ===== 测试9: 返回中心 =====
        print("\n=== 测试9: 返回中心位置=2048 ===")
        pos = 2048
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        write_reg(ser, servo_id, 42, [pos_low, pos_high])
        
        print("\n等待2秒...")
        time.sleep(2)
        
        # ===== 测试10: 读取电压 =====
        print("\n=== 测试10: 读取电压 (地址62) ===")
        data = read_reg(ser, servo_id, 62, 1)
        if data:
            voltage = data[0] / 10.0
            print(f"  电压: {voltage}V")
        
        ser.close()
        print("\n测试完成!")
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
    input("\n按Enter退出...")