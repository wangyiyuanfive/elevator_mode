"""
Modbus TCP客户端模块

    封装与SIEMENS S7-1200 PLC的Modbus TCP通信功能

功能:
    - Modbus TCP连接管理
    - 保持寄存器读写操作
    - 浮点数读写支持
    - 上下文管理器支持
    - 异常处理封装

依赖:
    - pymodbus >= 3.0.0
    - config （存储配置信息）
    - data_converter （做数据转换）

Author: Lenn Meng (Lingyu.Meng) & sth. from Internet
Date: 2026-03-14
Version: 1.0.0
"""

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

import config
from data_converter import (
    float_to_registers,
    registers_to_float,
    register_to_int16,
    register_to_uint16,
    int16_to_register,
    int32_to_registers,
    uint32_to_registers,
    registers_to_int32,
    registers_to_uint32,
    DataType
)


class ModbusTCPClient:

    def __init__(self, ip: str = None, port: int = None, unit_id: int = None):
        """
        初始化Modbus TCP客户端

        · ip: PLC的IP地址，默认使用config.py中的配置
        · port: Modbus端口，默认使用config.py中的配置
        · unit_id: 从站ID，默认使用config.py中的配置
        """
        self.ip = ip or config.PLC_IP
        self.port = port or config.PLC_PORT
        self.unit_id = unit_id or config.UNIT_ID
        self.client = None

    def connect(self) -> bool:
        """
        建立与PLC的连接

        连接成功返回True，否则返回False
        """
        try:
            self.client = ModbusTcpClient(
                host=self.ip,
                port=self.port,
                timeout=config.REQUEST_TIMEOUT
            )
            return self.client.connect()
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        """
        断开与PLC的连接

        安全关闭连接并释放资源
        """
        if self.client:
            self.client.close()
            self.client = None

    def read_holding_registers(self, address: int, count: int) -> list[int]:
        """
        读取保持寄存器（03功能字）
            address: 起始地址（0-based）
            count: 读取数量
            list[int]: 寄存器值列表，失败返回空列表
        """
        if not self.client:
            print("未连接到PLC")
            return []

        try:
            result = self.client.read_holding_registers(
                address=address,
                count=count,
                device_id=self.unit_id
            )
            if result.isError():
                print(f"读取寄存器错误: {result}")
                return []
            return list(result.registers)
        except ModbusException as e:
            print(f"Modbus异常: {e}")
            return []

    def write_single_register(self, address: int, value: int) -> bool:
        """
        写入单个寄存器（06功能字）

            address: 寄存器地址（0-based）
            value: 要写入的值（0-65535）
            bool: 成功返回True，失败返回False
        """
        if not self.client:
            print("未连接到PLC")
            return False

        try:
            result = self.client.write_register(
                address=address,
                value=value,
                device_id=self.unit_id
            )
            return not result.isError()
        except ModbusException as e:
            print(f"Modbus异常: {e}")
            return False

    def write_multiple_registers(self, address: int, values: list[int]) -> bool:
        """
        写入多个寄存器（16功能字）

            address: 起始地址（0-based）
            values: 要写入的值列表
            bool: 成功返回True，失败返回False
        """
        if not self.client:
            print("未连接到PLC")
            return False

        try:
            result = self.client.write_registers(
                address=address,
                values=values,
                device_id=self.unit_id
            )
            return not result.isError()
        except ModbusException as e:
            print(f"Modbus异常: {e}")
            return False

    def read_float(self, address: int) -> float | None:
        """
        读取浮点数（占用2个寄存器）
        一个地址是uint16，浮点数是32位，
        从指定地址读取2个连续寄存器并转换为浮点数


            address: 起始地址（0-based）
            float | None: 浮点数值，失败返回None
        """
        registers = self.read_holding_registers(address, 2)
        if len(registers) >= 2:
            return registers_to_float(registers)
        return None

    def write_float(self, address: int, value: float) -> bool:
        """
        写入浮点数（占用2个寄存器）
        将浮点数转换为2个寄存器并写入

            address: 起始地址（0-based）
            value: 浮点数值
            bool: 成功返回True，失败返回False
        """
        registers = float_to_registers(value)
        return self.write_multiple_registers(address, registers)

    def read_by_type(self, address: int, data_type: str):
        """
        按指定数据类型读取值

            address: 起始地址（0-based）
            data_type: 数据类型 ("int16", "uint16", "int32", "uint32", "float32")
            读取的值，失败返回None
        """
        count = DataType.REGISTER_COUNT.get(data_type, 1)
        registers = self.read_holding_registers(address, count)

        if not registers:
            return None

        if data_type == DataType.INT16:
            return register_to_int16(registers[0])
        elif data_type == DataType.UINT16:
            return register_to_uint16(registers[0])
        elif data_type == DataType.INT32:
            return registers_to_int32(registers)
        elif data_type == DataType.UINT32:
            return registers_to_uint32(registers)
        elif data_type == DataType.FLOAT32:
            return registers_to_float(registers)
        else:
            print(f"未知的数据类型: {data_type}")
            return None

    def write_by_type(self, address: int, value, data_type: str) -> bool:
        """
        按指定数据类型写入值

            address: 起始地址（0-based）
            value: 要写入的值
            data_type: 数据类型 ("int16", "uint16", "int32", "uint32", "float32")
            bool: 成功返回True，失败返回False
        """
        if data_type == DataType.INT16:
            # 有符号int16需要转换为无符号寄存器值
            return self.write_single_register(address, int16_to_register(int(value)))
        elif data_type == DataType.UINT16:
            return self.write_single_register(address, int(value))
        elif data_type == DataType.INT32:
            return self.write_multiple_registers(address, int32_to_registers(int(value)))
        elif data_type == DataType.UINT32:
            return self.write_multiple_registers(address, uint32_to_registers(int(value)))
        elif data_type == DataType.FLOAT32:
            return self.write_float(address, float(value))
        else:
            print(f"未知的数据类型: {data_type}")
            return False

    def __enter__(self):
        """
        自动建立连接
        """
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        自动断开连接
        """
        self.disconnect()
        return False