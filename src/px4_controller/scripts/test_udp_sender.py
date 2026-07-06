#!/usr/bin/env python3
"""UDP 广播测试发送器 — 用于诊断网络连通性"""
import socket
import time
import json
import argparse

MAGIC = b'TEL\x00'

def main():
    parser = argparse.ArgumentParser(description='UDP 遥测测试发送器')
    parser.add_argument('--ip', default='255.255.255.255', help='目标 IP (默认: 255.255.255.255)')
    parser.add_argument('--port', type=int, default=8881, help='目标端口 (默认: 8881)')
    parser.add_argument('--rate', type=int, default=5, help='发送频率 Hz')
    parser.add_argument('--no-magic', action='store_true', help='不添加 TEL 魔数头')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    addr = (args.ip, args.port)
    interval = 1.0 / args.rate
    count = 0

    print(f"[测试发送器] 目标: {args.ip}:{args.port}, 频率: {args.rate}Hz")
    print(f"[测试发送器] 开始发送... (Ctrl+C 退出)")

    try:
        while True:
            data = {
                "ts": time.time(),
                "connected": 1,
                "armed": 0,
                "mode": "TEST_MODE",
                "state": 0,
                "state_name": "WAITING",
                "pos_x": 0.0,
                "pos_y": 0.0,
                "pos_z": 0.0,
                "vel_x": 0.0,
                "vel_y": 0.0,
                "vel_z": 0.0,
                "initial_yaw": 1.57
            }
            payload = json.dumps(data).encode('utf-8')
            
            if args.no_magic:
                packet = payload
            else:
                packet = MAGIC + payload

            sock.sendto(packet, addr)
            count += 1
            print(f"  已发送 #{count}: {len(packet)} 字节", end='\r')
            time.sleep(interval)
    except KeyboardInterrupt:
        print(f"\n[测试发送器] 已停止，共发送 {count} 包")


if __name__ == '__main__':
    main()