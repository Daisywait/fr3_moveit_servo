#!/usr/bin/env python3
import time
import sys
import inspect
import pymodbus

# ⚠️ 端口配置
TARGET_PORT = '/dev/gripper' 

def test():
    print(f"📦 PyModbus 版本: {pymodbus.__version__}")
    print(f"🔍 正在测试端口: {TARGET_PORT}")

    # === 1. 初始化客户端 ===
    try:
        from pymodbus.client import ModbusSerialClient
        # Pymodbus 3.x+
        client = ModbusSerialClient(port=TARGET_PORT, stopbits=1, bytesize=8, parity='N', baudrate=115200, timeout=0.2)
    except ImportError:
        try:
            from pymodbus.client.sync import ModbusSerialClient
            # Pymodbus 2.x
            client = ModbusSerialClient(method='rtu', port=TARGET_PORT, stopbits=1, bytesize=8, parity='N', baudrate=115200, timeout=0.2)
        except ImportError:
            print("❌ 未找到 pymodbus")
            return

    if not client.connect():
        print(f"❌ 无法连接串口 {TARGET_PORT}")
        return

    # === 2. 动态参数适配 (适配 v3.11+) ===
    # 我们直接看函数签名来决定怎么传参
    sig = inspect.signature(client.read_input_registers)
    params = sig.parameters
    print(f"🛠️  API 签名参数: {list(params.keys())}")
    
    slave_id = 0x09
    kwargs = {}

    # 1. 确定从站ID的参数名
    if 'device_id' in params:
        print("👉 使用参数名: device_id (v3.11+)")
        kwargs['device_id'] = slave_id
    elif 'slave' in params:
        print("👉 使用参数名: slave")
        kwargs['slave'] = slave_id
    elif 'unit' in params:
        print("👉 使用参数名: unit")
        kwargs['unit'] = slave_id
    else:
        # 针对有些版本 kwargs 隐藏的情况
        print("⚠️ 未检测到 ID 参数，盲猜 'slave'...")
        kwargs['slave'] = slave_id

    # === 3. 执行测试 (强制使用关键字参数) ===
    # Pymodbus 3.11 要求 address 和 count 也必须显式指定参数名，不能只传位置参数
    try:
        print("👉 尝试读取状态寄存器...")
        
        # 兼容性调用：显式指定 address 和 count
        result = client.read_input_registers(address=0x07D0, count=3, **kwargs)
        
        if hasattr(result, 'isError') and result.isError():
            print(f"❌ 读取失败! 错误: {result}")
            print("   -> 可能是端口号错了，试试改脚本里的 TARGET_PORT 为 /dev/ttyUSB1")
        elif hasattr(result, 'registers'):
            print(f"✅ 读取成功！寄存器值: {result.registers}")
            g_status = (result.registers[0] >> 8) & 0xFF
            print(f"   夹爪状态 gOBJ: {g_status}")
            
            # 激活测试
            print("👉 发送复位 & 激活...")
            # write_registers 也强制使用关键字参数
            client.write_registers(address=0x03E8, values=[0x0000, 0x0000, 0x0000], **kwargs)
            time.sleep(0.5)
            client.write_registers(address=0x03E8, values=[0x0100, 0x0000, 0x0000], **kwargs)
            print("✅ 激活指令已发送 (观察夹爪是否动了)")
        else:
            print(f"❌ 未知响应: {result}")

    except Exception as e:
        print(f"❌ 发生异常: {e}")
        import traceback
        traceback.print_exc()

    client.close()

if __name__ == "__main__":
    test()