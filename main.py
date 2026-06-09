"""
Modbus TCP数据记录器 for S7-1200（其实其他的PLC也可以）

定时读取PLC寄存器数据并保存到CSV文件

"""

import csv
import time
import signal
import sys
from datetime import datetime

import config
from modbus_client import ModbusTCPClient


class DataLogger:

    def __init__(self, interval_ms: int = None):
        """
        初始化数据记录器
        interval_ms: 采集间隔（毫秒），使用config中的配置
        """
        self.interval_ms = interval_ms or config.SAMPLE_INTERVAL_MS
        self.interval_s = self.interval_ms / 1000.0
        self.client = ModbusTCPClient()
        self.csv_file = None
        self.csv_writer = None
        self.running = False

        # 生成CSV文件名（使用当前时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"data_log_{timestamp}.csv"

        # 准备CSV表头
        self.headers = ["时间戳"]
        for address, (data_type, description) in sorted(config.REGISTER_TYPE_MAP.items()):
            self.headers.append(f"{description}(Addr{address})")

    def start(self):
        """
        启动数据采集
        """
        # 连接PLC
        print(f"连接 PLC: {config.PLC_IP}:{config.PLC_PORT}")
        if not self.client.connect():
            print("连接失败！请检查PLC配置。")
            return

        print(f"连接成功！")

        # 创建CSV文件
        self.csv_file = open(self.csv_filename, 'w', newline='', encoding='utf-8-sig')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.headers)
        self.csv_file.flush()

        print(f"数据采集已启动")
        print(f"采集间隔: {self.interval_ms}ms")
        print(f"数据文件: {self.csv_filename}")
        # print(f"按 Ctrl+C 停止采集\n")

        self.running = True
        self._run_loop()

    def stop(self):
        """
        停止数据采集
        """
        self.running = False
        if self.csv_file:
            self.csv_file.close()
            print(f"\n数据已保存到: {self.csv_filename}")
        self.client.disconnect()
        print("连接已断开")

    def _run_loop(self):
        """
        采集循环
        """
        # 预计算寄存器范围，实现批量读取
        addresses = sorted(config.REGISTER_TYPE_MAP.keys())
        if not addresses:
            print("未配置寄存器地址")
            return

        self.reg_start = min(addresses)
        self.reg_end = max(addresses)
        # 计算最大需要的寄存器数量（考虑32位数据类型占用2个寄存器）
        from data_converter import DataType
        max_count = 0
        for addr, (dtype, _) in config.REGISTER_TYPE_MAP.items():
            count = DataType.REGISTER_COUNT.get(dtype, 1)
            max_count = max(max_count, addr - self.reg_start + count)
        self.reg_count = max_count

        next_time = time.perf_counter()

        while self.running:
            # 读取数据
            row = self._read_all_registers_batch()
            if row:
                self.csv_writer.writerow(row)
                self.csv_file.flush()
                # 打印数据
                self._print_row(row)

            # 计算下次执行时间（基于绝对时间）
            next_time += self.interval_s
            current_time = time.perf_counter()

            # 如果处理时间超过间隔，跳过到下一个周期
            if current_time > next_time:
                next_time = current_time + self.interval_s

            # 精确等待
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _print_row(self, row: list):
        parts = [str(row[0])]
        for value in row[1:]:
            parts.append(str(value))
        print(" | ".join(parts))

    def _read_all_registers_batch(self) -> list:
        """
        批量读取寄存器并解析
        一次性读取所有需要的寄存器，然后按数据类型解析

        """
        from data_converter import (
            register_to_int16, registers_to_int32, registers_to_uint32,
            registers_to_float, DataType
        )

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        row = [timestamp]

        # 批量读取所有寄存器
        all_registers = self.client.read_holding_registers(self.reg_start, self.reg_count)

        if not all_registers:
            return [timestamp] + [""] * len(config.REGISTER_TYPE_MAP)

        # 按配置解析每个变量
        for address in sorted(config.REGISTER_TYPE_MAP.keys()):
            data_type, description = config.REGISTER_TYPE_MAP[address]
            reg_index = address - self.reg_start

            try:
                if data_type == DataType.INT16:
                    value = register_to_int16(all_registers[reg_index])
                elif data_type == DataType.UINT16:
                    value = all_registers[reg_index]
                elif data_type == DataType.INT32:
                    value = registers_to_int32(all_registers[reg_index:reg_index + 2])
                elif data_type == DataType.UINT32:
                    value = registers_to_uint32(all_registers[reg_index:reg_index + 2])
                elif data_type == DataType.FLOAT32:
                    value = registers_to_float(all_registers[reg_index:reg_index + 2])
                else:
                    value = all_registers[reg_index]
                row.append(value)
            except (IndexError, ValueError) as e:
                row.append("")

        return row


def signal_handler(sig, frame):

    global logger
    print("\n\n正在停止采集...")
    if logger:
        logger.stop()
    sys.exit(0)


def main():

    global logger

    print("=" * 50)
    print("Modbus TCP 数据采集记录器")
    print("=" * 50)

    # 创建数据记录器（使用config中的采集间隔）
    logger = DataLogger()
    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)
    # 启动采集
    logger.start()


if __name__ == "__main__":
    logger = None
    main()
