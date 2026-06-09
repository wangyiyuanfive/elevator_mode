"""
数据转换器模块（测试ok了）

处理PLC中Real（IEEE 754浮点数）与寄存器的转换
主要是写入的时候用
读取的时候除非转换存储
但是还是以uint读进来比较好吧

    - 浮点数与16位寄存器互相转换
    - 整数与16位寄存器互相转换
    - 浮点数与十六进制字符串互相转换
    - 支持大端字节序（Big-Endian）

S7-1200 Real类型说明：
    - 32位浮点数，占用4字节 = 2个16位寄存器
    - 使用IEEE 754标准
    - 大端字节序（Big-Endian）
    - 示例: Real值 123.456 → Hex: 0x42F6E979 → 寄存器[0x42F6, 0xE979]

Author: Lenn Meng (Lingyu.Meng) & sth. from Internet
Date: 2026-03-14
"""

import struct


def float_to_registers(value: float) -> list[int]:
    """
    将浮点数转换为16位寄存器列表

    value: 浮点数值
    包含2个16位寄存器值的列表 [高16位, 低16位]

    Example:
        >>> float_to_registers(123.456)
        [17142, 59769]  # 0x42F6, 0xE979
    """
    # 使用大端序打包浮点数为4字节
    packed = struct.pack('>f', value)
    # 解包为2个无符号短整型（16位）
    registers = struct.unpack('>HH', packed)
    return list(registers)


def registers_to_float(registers: list[int]) -> float:
    """
    将16位寄存器列表转换为浮点数

    registers: 包含2个16位寄存器值的列表 [高16位, 低16位]
    返回浮点数值

    Example:
        >>> registers_to_float([17142, 59769])
        123.456
    """
    if len(registers) < 2:
        raise ValueError("需要至少2个寄存器来转换为浮点数")
    # 打包为4字节（大端序）
    packed = struct.pack('>HH', registers[0], registers[1])
    # 解包为浮点数
    return struct.unpack('>f', packed)[0]


# ============================================================
# 数据类型枚举
# ============================================================
class DataType:
    """数据类型枚举"""
    INT16 = "int16"       # 有符号16位整数 (1个寄存器)
    UINT16 = "uint16"     # 无符号16位整数 (1个寄存器)
    INT32 = "int32"       # 有符号32位整数 (2个寄存器)
    UINT32 = "uint32"     # 无符号32位整数 (2个寄存器)
    FLOAT32 = "float32"   # 32位浮点数 (2个寄存器)

    # 每种数据类型占用的寄存器数量
    REGISTER_COUNT = {
        "int16": 1,
        "uint16": 1,
        "int32": 2,
        "uint32": 2,
        "float32": 2
    }


# ============================================================
# 16位整数转换 (有符号/无符号)
# ============================================================
def register_to_int16(value: int) -> int:
    """
    将16位寄存器值转换为有符号16位整数

    value: 无符号16位寄存器值 (0-65535)
    Returns:有符号16位整数 (-32768 ~ 32767)
    """
    if value >= 0x8000:
        return value - 0x10000
    return value


def register_to_uint16(value: int) -> int:
    """
    将16位寄存器值转换为无符号16位整数

    value: 16位寄存器值
    Returns:无符号16位整数 (0 ~ 65535)
    """
    return value & 0xFFFF


def int16_to_register(value: int) -> int:
    """
    将有符号16位整数转换为寄存器值

    value: 有符号16位整数 (-32768 ~ 32767)
    Returns: 16位寄存器值 (0-65535)
    """
    if value < 0:
        return value + 0x10000
    return value


# ============================================================
# 32位整数转换 (有符号/无符号)
# ============================================================
def int32_to_registers(value: int) -> list[int]:
    """
    将有符号32位整数转换为16位寄存器列表

    value: 32位整数值
    Returns:包含2个16位寄存器值的列表 [高16位, 低16位]
    """
    packed = struct.pack('>i', value)
    registers = struct.unpack('>HH', packed)
    return list(registers)


def uint32_to_registers(value: int) -> list[int]:
    """
    将无符号32位整数转换为16位寄存器列表

    value: 无符号32位整数值
    Returns:包含2个16位寄存器值的列表 [高16位, 低16位]
    """
    packed = struct.pack('>I', value)
    registers = struct.unpack('>HH', packed)
    return list(registers)


def registers_to_int32(registers: list[int]) -> int:
    """
    将16位寄存器列表转换为有符号32位整数

    registers: 包含2个16位寄存器值的列表 [高16位, 低16位]
    Returns:有符号32位整数值
    """
    if len(registers) < 2:
        raise ValueError("需要至少2个寄存器来转换为32位整数")
    packed = struct.pack('>HH', registers[0], registers[1])
    return struct.unpack('>i', packed)[0]


def registers_to_uint32(registers: list[int]) -> int:
    """
    将16位寄存器列表转换为无符号32位整数

    registers: 包含2个16位寄存器值的列表 [高16位, 低16位]
    Returns:无符号32位整数值
    """
    if len(registers) < 2:
        raise ValueError("需要至少2个寄存器来转换为32位无符号整数")
    packed = struct.pack('>HH', registers[0], registers[1])
    return struct.unpack('>I', packed)[0]


# 保留旧函数名作为别名（向后兼容）
int_to_registers = int32_to_registers
registers_to_int = registers_to_int32


def float_to_hex(value: float) -> str:
    """
    将浮点数转换为十六进制字符串表示

    value: 浮点数值
    Returns:十六进制字符串，如 "0x42F6E979"

    Example:
        >>> float_to_hex(123.456)
        '0x42F6E979'
    """
    packed = struct.pack('>f', value)
    hex_value = struct.unpack('>I', packed)[0]
    return f"0x{hex_value:08X}"


def hex_to_float(hex_str: str) -> float:
    """
    将十六进制字符串转换为浮点数

    hex_str: 十六进制字符串，如 "0x42F6E979" 或 "42F6E979"
    Returns:浮点数值

    Example:
        >>> hex_to_float("0x42F6E979")
        123.456
    """
    # 移除0x前缀
    hex_str = hex_str.replace('0x', '').replace('0X', '')
    hex_value = int(hex_str, 16)
    packed = struct.pack('>I', hex_value)
    return struct.unpack('>f', packed)[0]


if __name__ == "__main__":
    # 测试转换函数
    print("=" * 50)
    print("数据转换测试")
    print("=" * 50)

    test_value = 123.456
    print(f"\n测试值: {test_value}")

    # 浮点数转寄存器
    regs = float_to_registers(test_value)
    print(f"寄存器值: {regs} (0x{regs[0]:04X}, 0x{regs[1]:04X})")

    # 浮点数转Hex
    hex_str = float_to_hex(test_value)
    print(f"Hex表示: {hex_str}")

    # 寄存器转浮点数
    recovered = registers_to_float(regs)
    print(f"恢复值: {recovered}")

    # Hex转浮点数
    from_hex = hex_to_float(hex_str)
    print(f"从Hex恢复: {from_hex}")