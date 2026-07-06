#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UDP 遥测数据接收端 — 上位机实时显示无人机状态
==============================================
监听 UDP 端口 8881，接收换行符分隔的 JSON 遥测数据。

使用方法:
    python3 telemetry_receiver.py [--port PORT] [--rate DISPLAY_RATE]
"""

import socket
import json
import argparse
import time
import os
import sys

# 尝试导入 colorama 用于彩色输出
try:
    from colorama import init, Fore, Style
    init(autoreset=True)
    COLOR = True
except ImportError:
    COLOR = False
    class Fore:
        RED = GREEN = YELLOW = BLUE = MAGENTA = CYAN = WHITE = RESET = ''
    class Style:
        BRIGHT = RESET_ALL = ''


class TelemetryReceiver:
    def __init__(self, port=8881, display_rate=10, verbose=False):
        self.port = port
        self.display_rate = display_rate
        self.verbose = verbose
        self.sock = None
        self.latest_data = {}
        self.packet_count = 0
        self.raw_count = 0
        self.last_display_time = 0
        self.start_time = time.time()

    def _setup_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.settimeout(0.5)

        print(f"{Fore.GREEN if COLOR else ''}[接收器] 已绑定 UDP 端口 {self.port}")
        print(f"[接收器] 等待遥测数据... (Ctrl+C 退出)")
        if self.verbose:
            print(f"[接收器] 详细模式: 显示所有原始数据包")

    def _parse_data(self, raw_data):
        """解析 UDP 数据：支持换行分隔的纯 JSON 或带 TEL 魔数头"""
        try:
            text = raw_data.decode('utf-8', errors='replace')
        except:
            return None

        # 方法1: 带 TEL 魔数头 (兼容旧格式)
        if raw_data[:4] == b'TEL\x00':
            return self._try_parse_json(raw_data[4:])

        # 方法2: 直接是 JSON
        return self._try_parse_json(raw_data)

    def _try_parse_json(self, data_bytes):
        try:
            text = data_bytes.decode('utf-8', errors='replace').strip()
            if text.startswith('{'):
                return json.loads(text)
        except:
            pass
        return None

    def _display_header(self):
        elapsed = time.time() - self.start_time
        pps = self.packet_count / elapsed if elapsed > 0 else 0
        print(f"\n{'=' * 78}")
        print(f"{Style.BRIGHT if COLOR else ''}[无人机遥测数据]  "
              f"运行: {elapsed:.0f}s | 有效包: {self.packet_count} "
              f"| PPS: {pps:.1f}")
        print(f"{'=' * 78}")

    def _display_connection_status(self):
        connected = self.latest_data.get('connected', 0)
        armed = self.latest_data.get('armed', 0)
        mode = self.latest_data.get('mode', 'UNKNOWN')
        state = self.latest_data.get('state', -1)
        state_name = self.latest_data.get('state_name', 'UNKNOWN')

        conn_str = (f"{Fore.GREEN if COLOR else ''}● 已连接" if connected
                    else f"{Fore.RED if COLOR else ''}● 未连接")
        arm_str = (f"{Fore.YELLOW if COLOR else ''}ARMED" if armed
                   else f"{Fore.WHITE if COLOR else ''}DISARMED")

        print(f"  MAVROS: {conn_str}{Style.RESET_ALL if COLOR else ''}  |  "
              f"{arm_str}{Style.RESET_ALL if COLOR else ''}  |  "
              f"模式: {Fore.CYAN if COLOR else ''}{mode}{Style.RESET_ALL if COLOR else ''}  |  "
              f"状态: {state_name} ({state})")

    def _display_position_velocity(self):
        px = self.latest_data.get('pos_x', 0)
        py = self.latest_data.get('pos_y', 0)
        pz = self.latest_data.get('pos_z', 0)
        vx = self.latest_data.get('vel_x', 0)
        vy = self.latest_data.get('vel_y', 0)
        vz = self.latest_data.get('vel_z', 0)
        yaw = self.latest_data.get('initial_yaw', 0)

        print(f"  {'─' * 76}")
        print(f"  {Style.BRIGHT if COLOR else ''}[位置/速度]{Style.RESET_ALL if COLOR else ''}")
        print(f"    X: {Fore.GREEN if COLOR else ''}{px:8.4f} m{Style.RESET_ALL if COLOR else ''}  "
              f"Y: {Fore.GREEN if COLOR else ''}{py:8.4f} m{Style.RESET_ALL if COLOR else ''}  "
              f"Z: {Fore.GREEN if COLOR else ''}{pz:8.4f} m{Style.RESET_ALL if COLOR else ''}")
        print(f"   VX: {Fore.YELLOW if COLOR else ''}{vx:8.4f} m/s{Style.RESET_ALL if COLOR else ''}  "
              f"VY: {Fore.YELLOW if COLOR else ''}{vy:8.4f} m/s{Style.RESET_ALL if COLOR else ''}  "
              f"VZ: {Fore.YELLOW if COLOR else ''}{vz:8.4f} m/s{Style.RESET_ALL if COLOR else ''}  "
              f"Yaw: {yaw:.2f} rad")

    def run(self):
        self._setup_socket()
        display_interval = 1.0 / self.display_rate

        while True:
            try:
                try:
                    raw_data, addr = self.sock.recvfrom(4096)
                except socket.timeout:
                    continue

                self.raw_count += 1

                if self.verbose:
                    print(f"\n[RAW #{self.raw_count}] 来自 {addr[0]}:{addr[1]}, "
                          f"{len(raw_data)} bytes, 前20字节: {raw_data[:20]!r}")

                parsed = self._parse_data(raw_data)
                if parsed is None:
                    if self.verbose:
                        print(f"  -> 无法解析为 JSON, 原始数据: {raw_data[:200]!r}")
                    continue

                self.latest_data = parsed
                self.packet_count += 1

                current_time = time.time()
                if current_time - self.last_display_time >= display_interval:
                    self.last_display_time = current_time
                    os.system('cls' if os.name == 'nt' else 'clear')
                    self._display_header()
                    self._display_connection_status()
                    self._display_position_velocity()

            except KeyboardInterrupt:
                print(f"\n\n{Fore.YELLOW if COLOR else ''}[接收器] 用户中断，退出...{Style.RESET_ALL if COLOR else ''}")
                break
            except Exception as e:
                print(f"\n{Fore.RED if COLOR else ''}[错误] {e}{Style.RESET_ALL if COLOR else ''}")

        self.cleanup()

    def cleanup(self):
        if self.sock:
            self.sock.close()
        elapsed = time.time() - self.start_time
        print(f"\n[接收器] 已停止。原始包: {self.raw_count}, 有效遥测包: {self.packet_count}, 运行: {elapsed:.1f}s")


def main():
    parser = argparse.ArgumentParser(description='UDP 无人机遥测数据接收器')
    parser.add_argument('--port', type=int, default=8881, help='UDP 监听端口 (默认: 8881)')
    parser.add_argument('--rate', type=int, default=10, help='终端显示刷新率 Hz (默认: 10)')
    parser.add_argument('-v', '--verbose', action='store_true', help='详细模式: 显示所有原始数据包')
    args = parser.parse_args()

    receiver = TelemetryReceiver(port=args.port, display_rate=args.rate, verbose=args.verbose)
    receiver.run()


if __name__ == '__main__':
    main()