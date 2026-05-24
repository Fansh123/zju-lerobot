#!/usr/bin/env python3
"""
读取舵机实际状态
"""

import serial
import time


def checksum(data):
    return (~sum(data)) & 0xFF


def read_register(ser, servo_id, address, length):
    """读取寄存器"""
    params = [address, length]
    data = [servo_id, 4, 0x2A] + params  # 0x2A = READ指令
    chk = checksum(data)
    packet = bytes([0xFF, 0xFF] + data + [chk])
    
    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.1)
    return ser.read(50)


def parse_response(resp, data_offset=5):
    """解析响应数据"""
    if not resp or len(resp) < 6:
        return None
    
    if resp[0] != 0xFF or resp[1] != 0xFF:
        return None
    
    length = resp[3]
    error = resp[4]
    
    if error != 0:
        print(f"    错误码: {error}")
    
    if length > 2 and len(resp) >= 5 + (length - 2):
        return resp[5:5 + (length - 2)]
    
    return None


def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    print("="*60)
    print("舵机状态诊断")
    print("="*60)
    
    try:
        ser = serial.Serial(port, 1000000, timeout=0.5)
        time.sleep(0.5)
        print(f"✓ 连接到 {port}\n")
        
        for servo_id in range(1, 7):
            print(f"=== 舵机 ID={servo_id} ===")
            
            # 读取扭矩状态 (地址40)
            resp = read_register(ser, servo_id, 40, 1)
            data = parse_response(resp)
            if data:
                torque = data[0]
                print(f"  扭矩使能: {torque} ({'启用' if torque else '禁用'})")
            else:
                print(f"  扭矩使能: 读取失败 ({resp.hex() if resp else '无响应'})")
            
            # 读取当前位置 (地址56或0x38)
            resp = read_register(ser, servo_id, 56, 2)
            data = parse_response(resp)
            if data and len(data) >= 2:
                pos = data[0] + (data[1] << 8)
                print(f"  当前位置: {pos}")
            else:
                print(f"  当前位置: 读取失败")
            
            # 读取目标位置 (地址42)
            resp = read_register(ser, servo_id, 42, 2)
            data = parse_response(resp)
            if data and len(data) >= 2:
                goal = data[0] + (data[1] << 8)
                print(f"  目标位置: {goal}")
            else:
                print(f"  目标位置: 读取失败")
            
            # 读取速度 (地址46)
            resp = read_register(ser, servo_id, 46, 2)
            data = parse_response(resp)
            if data and len(data) >= 2:
                speed = data[0] + (data[1] << 8)
                print(f"  速度: {speed}")
            else:
                print(f"  速度: 读取失败")
            
            # 读取加速度 (地址44)
            resp = read_register(ser, servo_id, 44, 1)
            data = parse_response(resp)
            if data:
                accel = data[0]
                print(f"  加速度: {accel}")
            else:
                print(f"  加速度: 读取失败")
            
            # 读取电压 (地址62)
            resp = read_register(ser, servo_id, 62, 1)
            data = parse_response(resp)
            if data:
                voltage = data[0] / 10.0
                print(f"  电压: {voltage}V")
            else:
                print(f"  电压: 读取失败")
            
            # 读取温度 (地址63)
            resp = read_register(ser, servo_id, 63, 1)
            data = parse_response(resp)
            if data:
                temp = data[0]
                print(f"  温度: {temp}°C")
            else:
                print(f"  温度: 读取失败")
            
            # 读取工作模式 (地址33)
            resp = read_register(ser, servo_id, 33, 1)
            data = parse_response(resp)
            if data:
                mode = data[0]
                mode_str = {
                    0: "位置模式",
                    1: "电机模式",
                    2: "步进模式"
                }.get(mode, f"未知({mode})")
                print(f"  工作模式: {mode_str}")
            else:
                print(f"  工作模式: 读取失败")
            
            print()
        
        ser.close()
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
    input("\n按Enter退出...")