#!/usr/bin/env python3
"""
串口检测工具
检查所有可用的串口，并尝试识别机械臂
"""

import serial
import serial.tools.list_ports
import sys

def list_available_ports():
    """列出所有可用的串口"""
    print("="*60)
    print("可用串口列表")
    print("="*60)
    
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("\n未发现任何串口！")
        print("\n请检查:")
        print("  1. USB线是否连接牢固")
        print("  2. USB线是否为数据线（非充电线）")
        print("  3. 尝试插到电脑后面的USB口")
        print("  4. 机械臂是否已通电")
        return []
    
    print(f"\n发现 {len(ports)} 个串口:\n")
    
    available = []
    for port in ports:
        print(f"  端口: {port.device}")
        print(f"  描述: {port.description}")
        print(f"  硬件ID: {port.hwid}")
        print(f"  制造商: {port.manufacturer}")
        print("-"*40)
        available.append(port)
    
    return available

def test_port(port_name):
    """测试指定串口"""
    print(f"\n测试端口: {port_name}")
    print("-"*40)
    
    for baudrate in [115200, 57600, 9600]:
        try:
            print(f"  尝试波特率: {baudrate}...", end=" ")
            ser = serial.Serial(port_name, baudrate, timeout=0.5)
            
            # 尝试发送一个读取命令
            ser.write(bytearray([0xFF, 0xFF, 0x01, 0x02, 0x02, 0x00, 0x05]))
            response = ser.read(10)
            ser.close()
            
            if response:
                print(f"✓ 收到响应!")
                print(f"  响应数据: {response.hex()}")
                return True, baudrate
            else:
                print("无响应")
                
        except Exception as e:
            print(f"✗ 失败: {e}")
    
    # 尝试检测CH340/FTDI
    try:
        ser = serial.Serial(port_name, 115200, timeout=0.5)
        ser.close()
        return True, 115200
    except:
        pass
    
    return False, None

def check_device_manager():
    """提示检查设备管理器"""
    print("\n" + "="*60)
    print("设备管理器检查")
    print("="*60)
    print("\n请手动检查设备管理器:")
    print("  1. 右键点击开始菜单 → 设备管理器")
    print("  2. 查看以下位置:")
    print("     - 端口(COM和LPT) ← 应该能看到机械臂")
    print("     - 其他设备 ← 可能有未识别的设备")
    print("     -通用串行总线控制器 ← 检查是否有问题设备")
    print("\n  3. 如果看到黄色感叹号:")
    print("     - 右键 → 更新驱动程序")
    print("     - 或手动安装驱动")

def main():
    print("="*60)
    print("SO-ARM 串口检测工具")
    print("="*60)
    
    # 列出可用串口
    ports = list_available_ports()
    
    if ports:
        print("\n" + "="*60)
        print("串口测试")
        print("="*60)
        
        for port in ports:
            success, baudrate = test_port(port.device)
            if success:
                print(f"\n✓ 端口 {port.device} 可能连接了设备!")
    
    # 检查设备管理器
    check_device_manager()
    
    # 常见问题提示
    print("\n" + "="*60)
    print("常见问题解决方案")
    print("="*60)
    print("""
    1. 【找不到串口】
       - 换一根USB线（有些线只能充电）
       - 插到电脑后面的USB口
       - 检查机械臂电源是否打开
    
    2. 【有串口但连接失败】
       - 检查波特率设置（通常115200）
       - 检查控制板型号
    
    3. 【驱动问题】
       - Waveshare板: 可能需要安装CH340驱动
       - 下载地址: https://www.wch.cn/downloads/CH341SER_ZIP.html
    
    4. 【Windows无法识别设备】
       - 尝试管理员权限运行
       - 检查USB供电是否足够
    """)

if __name__ == "__main__":
    main()
    input("\n按 Enter 键退出...")