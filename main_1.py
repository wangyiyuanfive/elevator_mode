"""
Modbus TCP数据记录器 + 128位BOOL解析 + 3位UINT输出 + 5秒时序缓存 + 电梯智能模式判断
"""
import csv
import time
import signal
import sys
from datetime import datetime
from collections import deque  # 系统自带，缓存历史数据

import config
from modbus_client import ModbusTCPClient


# ===================== 1. 工具函数：10进制 → 16位二进制字符串 =====================
def int_to_16bit_binary(value: int) -> str:
    try:
        value = max(0, min(65535, int(value)))
        return format(value, '016b')
    except:
        return "0000000000000000"


# ===================== 2. 128个IO变量命名（对照你的PLC变量表修改这里！） =====================
# 【注意】原始128位数据中包含备用点位，下面是完整映射，但备用点位会被过滤掉
IO_VAR_NAMES_RAW = [

    # VALUE1 (DB0.7~0.0 + DB1.7~1.0) 16位
    "8层上行呼梯按钮", "7层上行呼梯按钮", "6层上行呼梯按钮", "5层上行呼梯按钮",
    "4层上行呼梯按钮", "3层上行呼梯按钮", "2层上行呼梯按钮", "1层上行呼梯按钮",
    "8层下行呼梯按钮", "7层下行呼梯按钮", "6层下行呼梯按钮", "5层下行呼梯按钮",
    "4层下行呼梯按钮", "3层下行呼梯按钮", "2层下行呼梯按钮", "9层上行呼梯按钮",

    # VALUE2 (DB2.7~2.0 + DB3.7~3.0) 16位
    "1号梯轿内选层按钮6", "1号梯轿内选层按钮5", "1号梯轿内选层按钮4", "1号梯轿内选层按钮3",
    "1号梯轿内选层按钮2", "1号梯轿内选层按钮1", "10层下行呼梯按钮", "9层下行呼梯按钮",
    "1号梯检修信号", "1号梯光幕信号", "1号梯轿内关门按钮", "1号梯轿内开门按钮",
    "1号梯轿内选层按钮10", "1号梯轿内选层按钮9", "1号梯轿内选层按钮8", "1号梯轿内选层按钮7",

    # VALUE3 (DB4.7~4.0 + DB5.7~5.0) 16位
    "1号梯7楼层门锁信号", "1号梯6楼层门锁信号", "1号梯5楼层门锁信号", "1号梯4楼层门锁信号",
    "1号梯3楼层门锁信号", "1号梯2楼层门锁信号", "1号梯1楼层门锁信号", "1号梯轿厢门锁信号",
    "1号梯上端站第1限位", "1号梯下平层信号", "1号梯上平层信号","1号梯关门到位",
    "1号梯开门到位", "1号梯10楼层门锁信号", "1号梯9楼层门锁信号","1号梯8楼层门锁信号",

    # VALUE4 (DB6.7~6.0 + DB7.7~7.0) 16位
    "2号梯轿内选层按钮5", "2号梯轿内选层按钮4", "2号梯轿内选层按钮3", "2号梯轿内选层按钮2",
    "2号梯轿内选层按钮1", "1号梯下端站第2限位", "1号梯下端站第1限位", "1号梯上端站第2限位",
    "2号梯光幕信号","2号梯轿内关门按钮", "2号梯轿内开门按钮", "2号梯轿内选层按钮10",
    "2号梯轿内选层按钮9","2号梯轿内选层按钮8", "2号梯轿内选层按钮7", "2号梯轿内选层按钮6",

    # VALUE5 (DB8.7~8.0 + DB9.7~9.0) 16位
    "2号梯6楼层门锁信号","2号梯5楼层门锁信号", "2号梯4楼层门锁信号", "2号梯3楼层门锁信号",
    "2号梯2楼层门锁信号","2号梯1楼层门锁信号", "2号梯轿厢门锁信号", "2号梯检修信号",
    "2号梯下平层信号", "2号梯上平层信号", "2号梯关门到位", "2号梯开门到位",
    "2号梯10楼层门锁信号","2号梯9楼层门锁信号","2号梯8楼层门锁信号","2号梯7楼层门锁信号",

    # VALUE6 (DB10.7~10.0 + DB11.7~11.0) 16位
    "3号梯轿内选层按钮4", "3号梯轿内选层按钮3", "3号梯轿内选层按钮2", "3号梯轿内选层按钮1",
    "2号梯下端站2限位", "2号梯下端站1限位", "2号梯上端站2限位", "2号梯上端站1限位",
    "3号梯轿内关门按钮", "3号梯轿内开门按钮", "3号梯轿内选层按钮10", "3号梯轿内选层按钮9",
    "3号梯轿内选层按钮8", "3号梯轿内选层按钮7", "3号梯轿内选层按钮6", "3号梯轿内选层按钮5",

    # ==================== VALUE7 (DBX12.7~12.0 + DBX13.7~13.0) 16位 ====================
    "3号梯5楼层门锁信号", "3号梯4楼层门锁信号", "3号梯3楼层门锁信号", "3号梯2楼层门锁信号",
    "3号梯1楼层门锁信号", "3号梯轿厢门锁信号", "3号梯检修信号", "3号梯光幕信号",
    "3号梯上平层信号", "3号梯关门到位", "3号梯开门到位", "3号梯10楼层门锁信号",
    "3号梯9楼层门锁信号", "3号梯8楼层门锁信号", "3号梯7楼层门锁信号", "3号梯6楼层门锁信号",

    # ==================== VALUE8 (DBX14.7~14.0 + DBX15.7~15.0) 16位 ====================
    "备用点位1", "备用点位2","自动运行信号","3号梯下端站2限位",
    "3号梯下端站1限位", "3号梯上端站2限位", "3号梯上端站1限位", "3号梯下平层信号",
    "备用点位3", "备用点位4", "备用点位5", "备用点位6",
    "备用点位7", "备用点位8", "备用点位9", "备用点位10"
]

# ===================== 过滤备用点位：生成有效IO变量名列表和索引 =====================
# 有效索引：记录原始128位中哪些位置是有效IO（非备用点位）
VALID_IO_INDICES = [i for i, name in enumerate(IO_VAR_NAMES_RAW) if not name.startswith("备用点位")]
# 有效IO变量名列表（用于CSV表头）
IO_VAR_NAMES = [IO_VAR_NAMES_RAW[i] for i in VALID_IO_INDICES]


# ===================== 3. 电梯运行模式 =====================
class ElevatorMode:
    IDLE = "空闲模式"
    UP_PEAK = "上行高峰模式"
    DOWN_PEAK = "下行高峰模式"
    INTERFLOOR = "层间交通模式"
    FIRE = "消防模式"
    MEETING = "会议活动突发流"


# ===================== 主程序 =====================
class DataLogger:
    def __init__(self, interval_ms: int = None):
        self.interval_ms = interval_ms or config.SAMPLE_INTERVAL_MS
        self.interval_s = self.interval_ms / 1000.0
        self.client = ModbusTCPClient()
        self.csv_file = None
        self.csv_writer = None
        self.running = False

        # ===================== 核心：缓存5秒数据（100ms采集一次 → 50条） =====================
        self.history_cache = deque(maxlen=int(5 / self.interval_s))

        # CSV文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"data_log_{timestamp}.csv"

        # 【关键修改】CSV表头：时间戳 + 128个BOOL变量 + 3个UINT变量 + 电梯模式
        self.headers = ["时间戳"]
        self.headers.extend(IO_VAR_NAMES)  # 128个BOOL变量名
        self.headers.extend(["1号梯当前载重量", "2号梯当前载重量", "3号梯当前载重量"])  # 3个UINT变量
        self.headers.append("电梯运行模式")

    def start(self):
        print(f"连接 PLC: {config.PLC_IP}:{config.PLC_PORT}")
        if not self.client.connect():
            print("连接失败！")
            return
        print("连接成功！")

        self.csv_file = open(self.csv_filename, 'w', newline='', encoding='utf-8-sig')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.headers)
        self.csv_file.flush()

        print(f"采集启动 | 5秒时序分析 | 智能模式判断")
        print(f"CSV输出格式: {len(self.headers)}列 = 1(时间戳)+{len(IO_VAR_NAMES)}(BOOL有效IO)+3(UINT)+1(模式) | 已过滤10个备用点位")
        self.running = True
        self._run_loop()

    def stop(self):
        self.running = False
        if self.csv_file:
            self.csv_file.close()
            print(f"\n数据已保存: {self.csv_filename}")
        self.client.disconnect()
        print("断开连接")

    # ===================== 【核心】解析8个VALUE → 有效IO布尔变量(0/1)，过滤备用点位 =====================
    def parse_128_bits(self, value1_to_8):
        binary_str = ""
        for val in value1_to_8:
            binary_str += int_to_16bit_binary(val)
        # 【关键修改】只提取有效索引位置的布尔值，过滤掉备用点位
        bool_values = []
        io_states = {}
        for idx, i in enumerate(VALID_IO_INDICES):  # 同时获取有效索引的序号和原始位置
            bit_value = 1 if binary_str[i] == '1' else 0
            bool_values.append(bit_value)
            io_states[IO_VAR_NAMES[idx]] = (binary_str[i] == '1')  # 使用idx索引IO_VAR_NAMES
        return bool_values, io_states

    # ===================== 【核心】基于5秒数据 → 判断电梯模式 =====================
    def judge_mode(self):
        if len(self.history_cache) < 10:
            return ElevatorMode.IDLE

        # 统计5秒内信号
        fire_count = 0
        up_calls_total = 0
        down_calls_total = 0
        car_calls_total = 0
        idle_count = 0
        meeting_count = 0

        for io in self.history_cache:
            # 消防信号（最高优先级）
            if io.get("消防模式", False):
                fire_count += 1
            # 统计外呼
            up = sum([io.get(f"{i}楼外呼上", False) for i in range(1, 11)])
            down = sum([io.get(f"{i}楼外呼下", False) for i in range(1, 11)])
            car = sum([io.get(f"轿内{i}楼", False) for i in range(1, 11)])
            up_calls_total += up
            down_calls_total += down
            car_calls_total += car
            if up + down + car == 0:
                idle_count += 1
            # 会议模式判断：短时间内多楼层集中呼梯
            if (up + down + car) > 5:
                meeting_count += 1

        # 规则判断
        if fire_count > 0:
            return ElevatorMode.FIRE
        if meeting_count > len(self.history_cache) * 0.3:
            return ElevatorMode.MEETING
        if idle_count > len(self.history_cache) * 0.8:
            return ElevatorMode.IDLE
        if up_calls_total > down_calls_total * 2:
            return ElevatorMode.UP_PEAK
        if down_calls_total > up_calls_total * 2:
            return ElevatorMode.DOWN_PEAK
        return ElevatorMode.INTERFLOOR

    def _run_loop(self):
        addresses = sorted(config.REGISTER_TYPE_MAP.keys())
        if not addresses:
            print("未配置寄存器")
            return
        self.reg_start = min(addresses)
        from data_converter import DataType
        max_count = 0
        for addr, (dtype, _) in config.REGISTER_TYPE_MAP.items():
            max_count = max(max_count, addr - self.reg_start + DataType.REGISTER_COUNT.get(dtype, 1))
        self.reg_count = max_count

        next_time = time.perf_counter()
        while self.running:
            row = self._read_all_registers_batch()
            if row:
                self.csv_writer.writerow(row)
                self.csv_file.flush()
                self._print_row(row)

            next_time += self.interval_s
            if time.perf_counter() > next_time:
                next_time = time.perf_counter()
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _print_row(self, row):
        # 控制台只打印关键信息，避免刷屏
        # 【修改】BOOL值数量现在是len(IO_VAR_NAMES)个，不再是固定的128个
        bool_count = len(IO_VAR_NAMES)
        print(
            f"{row[0]} | 模式: {row[-1]} | BOOL信号: {sum(row[1:1+bool_count])}个激活 | VALUE9: {row[1+bool_count]}, VALUE10: {row[2+bool_count]}, VALUE11: {row[3+bool_count]}")

    # ===================== 数据读取+解析主函数 =====================
    def _read_all_registers_batch(self):
        from data_converter import register_to_int16, DataType
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        row = [timestamp]  # 第一列：时间戳
        reg_data = self.client.read_holding_registers(self.reg_start, self.reg_count)
        if not reg_data:
            # 数据读取失败时填充默认值
            bool_defaults = [0] * len(IO_VAR_NAMES)  # 【修改】使用有效IO数量，不再是128
            uint_defaults = [""] * 3
            return row + bool_defaults + uint_defaults + ["未知"]

        # 读取所有VALUE
        value1_to_8 = []
        value9_to_11 = []
        parsed_values = {}
        for addr in sorted(config.REGISTER_TYPE_MAP.keys()):
            dtype, desc = config.REGISTER_TYPE_MAP[addr]
            idx = addr - self.reg_start
            try:
                val = register_to_int16(reg_data[idx]) if dtype == DataType.INT16 else reg_data[idx]
                parsed_values[desc] = val
                if desc in [f"VALUE{i}" for i in range(1, 9)]:
                    value1_to_8.append(val)
                elif desc in [f"VALUE{i}" for i in range(9, 12)]:
                    value9_to_11.append(val)
            except:
                parsed_values[desc] = ""

        # 解析128位布尔变量（0/1值列表）和IO状态字典
        bool_values, io_states = self.parse_128_bits(value1_to_8)
        # 存入5秒缓存
        self.history_cache.append(io_states)
        # 判断模式
        mode = self.judge_mode()
        # 【关键修改】组装CSV行：时间戳 + 有效BOOL值(已过滤备用点位) + 3个UINT + 模式
        row.extend(bool_values)  # 有效BOOL值（0/1），数量为len(IO_VAR_NAMES)
        row.extend(value9_to_11)  # 3个UINT值（VALUE9~11）
        row.append(mode)  # 电梯运行模式
        return row


# ===================== 信号处理 =====================
def signal_handler(sig, frame):
    global logger
    print("\n停止采集...")
    if logger:
        logger.stop()
    sys.exit(0)


# ===================== 主入口 =====================
def main():
    global logger
    print("=" * 50)
    print("电梯智能数据采集&模式识别系统")
    print("=" * 50)
    logger = DataLogger()
    signal.signal(signal.SIGINT, signal_handler)
    logger.start()


if __name__ == "__main__":
    logger = None
    main()