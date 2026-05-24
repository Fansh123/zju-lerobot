#!/usr/bin/env python3
"""
使用多种可能的地址测试舵机
"""

import serial
import time


def checksum(data):
    return (~sum(data)) & 0xFF


def send_write(ser, servo_id, address, values, wait_response=False):
    """发送写指令"""
    if isinstance(values, int):
        values = [values]
    
    params = [address] + values
    length = len(params) + 2
    data = [servo_id, length, 0x28] + params
    chk = checksum(data)
    packet = bytes([0xFF, 0xFF] + data + [chk])
    
    print(f"  发送: {packet.hex()}")
    ser.reset_input_buffer()
    ser.write(packet)
    
    if wait_response:
        time.sleep(0.1)
        resp = ser.read(20)
        print(f"  响应: {resp.hex() if resp else '(无)'}")
        return resp
    return None


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("多地址测试")
    print("="*60)
    
    try:
        ser = serial.Serial(port, 1000000, timeout=0.5)
        time.sleep(0.5)
        print(f"✓ 连接到 {port}\n")
        
        servo_id = 1
        
        # ===== 测试1: 启用扭矩 =====
        print("=== 测试1: 启用扭矩 (地址40) ===")
        send_write(ser, servo_id, 40, [1], wait_response=True)
        
        # ===== 测试2: 设置加速度 (地址44) =====
        print("\n=== 测试2: 设置加速度 (地址44) ===")
        send_write(ser, servo_id, 44, [50], wait_response=True)  # 加速度50
        
        # ===== 测试3: 设置速度 (地址46) =====
        print("\n=== 测试3: 设置速度 (地址46) ===")
        speed = 500
        spd_low = speed & 0xFF
        spd_high = (speed >> 8) & 0xFF
        send_write(ser, servo_id, 46, [spd_low, spd_high], wait_response=True)
        
        # ===== 测试4: 使用地址0x2A写目标位置 =====
        print("\n=== 测试4: 写目标位置到地址0x2A (低位) ===")
        pos = 1500
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        
        # 先写低位
        send_write(ser, servo_id, 0x2A, [pos_low], wait_response=True)
        # 再写高位
        print("\n=== 测试4b: 写目标位置到地址0x2B (高位) ===")
        send_write(ser, servo_id, 0x2B, [pos_high], wait_response=True)
        
        print("\n等待2秒...")
        time.sleep(2)
        
        # ===== 测试5: 使用地址42写目标位置 =====
        print("\n=== 测试5: 写目标位置到地址42 (2字节) ===")
        pos = 2500
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        send_write(ser, servo_id, 42, [pos_low, pos_high], wait_response=True)
        
        print("\n等待2秒...")
        time.sleep(2)
        
        # ===== 测试6: 使用地址0x74写目标位置 =====
        print("\n=== 测试6: 写目标位置到地址0x74 ===")
        pos = 2048
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        send_write(ser, servo_id, 0x74, [pos_low, pos_high], wait_response=True)
        
        print("\n等待2秒...")
        time.sleep(2)
        
        # ===== 测试7: 使用Reg Write指令 =====
        print("\n=== 测试7: 使用REG WRITE指令 ===")
        # REG WRITE = 0x29, 写入缓存但不立即执行
        pos = 1500
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        
        params = [42, pos_low, pos_high]
        length = len(params) + 2
        data = [servo_id, length, 0x29] + params
        chk = checksum(data)
        packet = bytes([0xFF, 0xFF] + data + [chk])
        print(f"  发送REG WRITE: {packet.hex()}")
        ser.write(packet)
        time.sleep(0.1)
        
        # ACTION指令执行
        print("\n=== 发送ACTION指令 ===")
        data = [servo_id, 2, 0x05]  # ACTION = 0x05
        chk = checksum(data)
        packet = bytes([0xFF, 0xFF] + data + [chk])
        print(f"  发送ACTION: {packet.hex()}")
        ser.write(packet)
        
        print("\n等待2秒...")
        time.sleep(2)
        
        # ===== 测试8: 同步写 =====
        print("\n=== 测试8: SYNC WRITE (同步写) ===")
        # SYNC WRITE = 0x83
        # 格式: FF FF FE L 0x83 addr len [id data...] checksum
        addr = 42
        data_len = 2  # 每个舵机2字节数据
        
        pos = 2048
        pos_low = pos & 0xFF
        pos_high = (pos >> 8) & 0xFF
        
        # 构建参数
        params = [addr, data_len]  # 起始地址, 数据长度
        for i in range(1, 7):  # 舵机1-6
            params.extend([i, pos_low, pos_high])
        
        length = len(params) + 2
        data = [0xFE, length, 0x83] + params  # 0xFE = 广播ID
        chk = checksum(data)
        packet = bytes([0xFF, 0xFF] + data + [chk])
        print(f"  发送SYNC WRITE: {packet.hex()}")
        ser.write(packet)
        
        print("\n等待2秒...")
        time.sleep(2)
        
        ser.close()
        print("\n测试完成!")
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
    input("\n按Enter退出...")