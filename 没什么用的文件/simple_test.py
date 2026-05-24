#!/usr/bin/env python3
"""
最简化的通信测试 - 尝试多种指令格式
"""

import serial
import time

def test_simple_commands(port='COM18'):
    """发送最简单直接的指令"""
    
    print("="*60)
    print("最简化通信测试")
    print("="*60)
    
    for baudrate in [1000000, 115200]:
        print(f"\n\n{'='*60}")
        print(f"测试波特率: {baudrate}")
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
            print(f"✓ 串口 {port} 打开成功 @ {baudrate}")
            
            # 清除缓冲区
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            # 测试各种指令格式
            tests = [
                # 格式1: 标准Feetech读指令 [FF FF ID LEN INST ADDR_L ADDR_H CHECK]
                ("Feetech读位置", [0xFF, 0xFF, 0x01, 0x04, 0x2A, 0x38, 0x00, 0x00]),
                
                # 格式2: 带正确校验和的读指令
                ("Feetech读位置(校验)", [0xFF, 0xFF, 0x01, 0x04, 0x2A, 0x38, 0x00, 0x00]),
                
                # 格式3: 简单ping [FF FF ID LEN INST CHECK]
                ("Ping舵机1", [0xFF, 0xFF, 0x01, 0x02, 0x01, 0x00]),
                
                # 格式4: 广播ping
                ("Ping广播", [0xFF, 0xFF, 0xFE, 0x02, 0x01, 0x00]),
                
                # 格式5: 写扭矩使能
                ("写扭矩使能", [0xFF, 0xFF, 0x01, 0x05, 0x28, 0x28, 0x01, 0x00]),
                
                # 格式6: 舵机移动到中心
                ("移动舵机1到中心", [0xFF, 0xFF, 0x01, 0x07, 0x28, 0x2A, 0x00, 0x08, 0xF4, 0x01, 0x00]),
                
                # 格式7: 不带校验和的移动
                ("直接移动", [0xFF, 0xFF, 0x01, 0x07, 0x28, 0x2A, 0x00, 0x08, 0x00, 0x00]),
                
                # 格式8: SCS协议格式
                ("SCS协议读", [0x55, 0x55, 0x01, 0x03, 0x24, 0x00]),
                
                # 格式9: 只发header
                ("仅Header", [0xFF, 0xFF]),
                
                # 格式10: 查询固件版本
                ("固件版本", [0xFF, 0xFF, 0x01, 0x03, 0x28, 0x00, 0x00]),
            ]
            
            for name, cmd in tests:
                ser.reset_input_buffer()
                
                cmd_bytes = bytes(cmd)
                print(f"\n{name}:")
                print(f"  发送: {cmd_bytes.hex()}")
                
                ser.write(cmd_bytes)
                time.sleep(0.5)
                
                # 读取所有可用数据
                response = ser.read(50)
                if response:
                    print(f"  收到: {response.hex()}")
                    print(f"  长度: {len(response)}")
                else:
                    print(f"  收到: (无)")
                
                time.sleep(0.2)
            
            ser.close()
            
        except Exception as e:
            print(f"错误: {e}")

def test_with_scanning(port='COM18'):
    """扫描所有舵机ID"""
    print("\n\n" + "="*60)
    print("扫描舵机ID (1-10)")
    print("="*60)
    
    try:
        ser = serial.Serial(port, 1000000, timeout=0.5)
        time.sleep(0.3)
        
        for servo_id in range(1, 11):
            ser.reset_input_buffer()
            
            # 尝试读取位置
            cmd = bytes([0xFF, 0xFF, servo_id, 0x04, 0x2A, 0x38, 0x00, 0x00])
            ser.write(cmd)
            time.sleep(0.2)
            
            response = ser.read(20)
            if response and len(response) >= 6:
                if response[2] == servo_id:  # 检查返回的ID
                    print(f"✓ 舵机 ID={servo_id} 响应!")
                    print(f"  数据: {response.hex()}")
            else:
                print(f"  ID={servo_id}: 无响应")
        
        ser.close()
        
    except Exception as e:
        print(f"错误: {e}")

def main():
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM18'
    
    test_simple_commands(port)
    test_with_scanning(port)
    
    print("\n" + "="*60)
    print("测试完成")
    print("="*60)
    print("\n如果所有测试都没有响应:")
    print("1. 检查舵机是否正确连接到控制板")
    print("2. 检查电源是否足够 (12V 2A+)")
    print("3. 尝试重新插拔USB和电源")

if __name__ == "__main__":
    main()
    input("\n按Enter退出...")