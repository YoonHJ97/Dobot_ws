#!/usr/bin/env python3
"""
MG400 TCP 위치 실시간 모니터.

포트 30004(실시간 피드백)에 연결해 tool_vector_actual(실제 TCP 포즈)를
주기적으로 출력한다. ROS2 빌드와 무관하게 동작한다.

사용법:
    python3 tcp_monitor.py            # 기본 IP 192.168.1.6
    python3 tcp_monitor.py 192.168.1.6
"""
import sys
import time
import socket

import numpy as np

from dobot_api import MyType  # 피드백 패킷 dtype (tool_vector_actual 포함)

FEED_PORT = 30004
MAGIC = 0x123456789abcdef  # 정상 패킷 식별값(test_value)


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.6"

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3)
    s.connect((ip, FEED_PORT))
    print(f"[연결됨] {ip}:{FEED_PORT}  (Ctrl+C 로 종료)\n")

    buf = b""
    last_print = 0.0
    try:
        while True:
            buf += s.recv(1440)
            if len(buf) < 1440:
                continue
            # 1440바이트 한 패킷씩 처리
            packet, buf = buf[:1440], buf[1440:]
            data = np.frombuffer(packet, dtype=MyType)
            if hex(data["test_value"][0]) != hex(MAGIC):
                # 패킷 경계가 어긋남 -> 버퍼 초기화 후 재동기화
                buf = b""
                continue

            # 10Hz 로만 출력
            now = time.time()
            if now - last_print < 0.1:
                continue
            last_print = now

            x, y, z, rx, ry, rz = data["tool_vector_actual"][0]
            print(
                f"X:{x:8.2f}  Y:{y:8.2f}  Z:{z:8.2f}  "
                f"Rx:{rx:7.2f}  Ry:{ry:7.2f}  Rz:{rz:7.2f}",
                end="\r",
                flush=True,
            )
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        s.close()


if __name__ == "__main__":
    main()
