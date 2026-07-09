#!/usr/bin/env python3
"""Sniff Livox MID360 packets and inspect livox time_type fields.

Use this to verify whether the driver is likely using packet timestamps
(PTP/GPS) or host fallback timestamps.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import time
from collections import Counter, defaultdict
from pathlib import Path

ETH_P_ALL = 0x0003
ETH_HDR_LEN = 14
IPV4_PROTO_UDP = 17
VLAN_TYPES = {0x8100, 0x88A8}
TIME_TYPES = {
    0: "NoSync (driver fallback to high_resolution_clock::now)",
    1: "PTP/GPTP",
    2: "GPS",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check Livox MID360 UDP time_type values from raw packets."
    )
    parser.add_argument(
        "--config",
        default="/home/user/ws/fastlio-go2w/humble_ws/src/fastlio_go2w_bringup/config/MID360_config.json",
        help="Path to MID360_config.json (default: repo config).",
    )
    parser.add_argument(
        "--iface",
        default="",
        help="Network interface to bind (empty means any interface).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Capture duration in seconds.",
    )
    parser.add_argument(
        "--max-packets",
        type=int,
        default=200,
        help="Stop after this many matching packets.",
    )
    parser.add_argument(
        "--point-port",
        type=int,
        default=None,
        help="Override host point-data UDP port (default from config).",
    )
    parser.add_argument(
        "--imu-port",
        type=int,
        default=None,
        help="Override host IMU-data UDP port (default from config).",
    )
    parser.add_argument(
        "--udp-dump-limit",
        type=int,
        default=20,
        help="Print per-packet lines for first N matching packets.",
    )
    return parser.parse_args()


def load_ports(config_path: str, point_port: int | None, imu_port: int | None) -> tuple[int, int]:
    default_point = 56301
    default_imu = 56401
    if point_port is not None and imu_port is not None:
        return point_port, imu_port

    if not Path(config_path).is_file():
        return point_port or default_point, imu_port or default_imu

    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        host = cfg.get("MID360", {}).get("host_net_info", {})
        cfg_point = int(host.get("point_data_port", point_port or default_point))
        cfg_imu = int(host.get("imu_data_port", imu_port or default_imu))
        return cfg_point, cfg_imu
    except Exception:
        return point_port or default_point, imu_port or default_imu


def parse_ethernet(frame: bytes) -> tuple[int, bytes] | None:
    if len(frame) < ETH_HDR_LEN:
        return None
    etype = int.from_bytes(frame[12:14], "big")
    offset = ETH_HDR_LEN
    if etype in VLAN_TYPES:
        if len(frame) < ETH_HDR_LEN + 4:
            return None
        etype = int.from_bytes(frame[16:18], "big")
        offset = ETH_HDR_LEN + 4
    if etype != 0x0800:
        return None
    return offset, frame[offset:]


def parse_ipv4(payload: bytes) -> tuple[int, int, bytes] | None:
    if len(payload) < 20:
        return None
    ihl = (payload[0] & 0x0F) * 4
    if ihl < 20 or (payload[0] >> 4) != 4:
        return None
    if len(payload) < ihl + 8:
        return None
    if payload[9] != IPV4_PROTO_UDP:
        return None

    udp_start = ihl
    sport, dport = struct.unpack("!HH", payload[udp_start : udp_start + 4])
    return sport, dport, payload[udp_start + 8 :]


def parse_livox(packet: bytes) -> dict[str, int] | None:
    if len(packet) < 36:
        return None
    return {
        "version": packet[0],
        "length": int.from_bytes(packet[1:3], "little"),
        "time_interval": int.from_bytes(packet[3:5], "little"),
        "dot_num": int.from_bytes(packet[5:7], "little"),
        "udp_cnt": int.from_bytes(packet[7:9], "little"),
        "frame_cnt": packet[9],
        "data_type": packet[10],
        "time_type": packet[11],
        "timestamp": int.from_bytes(packet[28:36], "little"),
    }


def stream_name(port: int, point_port: int, imu_port: int) -> str:
    if port == point_port:
        return "point"
    if port == imu_port:
        return "imu"
    return "other"


def main() -> int:
    args = parse_args()
    point_port, imu_port = load_ports(args.config, args.point_port, args.imu_port)

    try:
        sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(ETH_P_ALL))
    except PermissionError as err:
        print(f"[error] socket open failed: {err}")
        print("       Run with root privileges or grant CAP_NET_RAW to this interpreter.")
        return 1

    if args.iface:
        sock.bind((args.iface, ETH_P_ALL))
    sock.settimeout(0.5)

    print(f"[info] Using point_host_port={point_port}, imu_host_port={imu_port}")
    print(f"[info] Config: {args.config}")
    print(f"[info] interface={args.iface or 'any'} duration={args.duration}s max={args.max_packets}")

    matched = 0
    by_stream = defaultdict(lambda: {"count": 0, "time_types": Counter(), "ports": set()})
    deadline = time.monotonic() + args.duration

    try:
        while matched < args.max_packets and time.monotonic() < deadline:
            try:
                frame = sock.recv(65535)
            except socket.timeout:
                continue

            eth = parse_ethernet(frame)
            if eth is None:
                continue
            _, ip_payload = eth

            ipv4 = parse_ipv4(ip_payload)
            if ipv4 is None:
                continue
            sport, dport, udp_payload = ipv4
            if sport not in (point_port, imu_port) and dport not in (point_port, imu_port):
                continue

            livox = parse_livox(udp_payload)
            if livox is None:
                continue

            stream = stream_name(dport if dport in (point_port, imu_port) else sport, point_port, imu_port)
            by_stream[stream]["count"] += 1
            by_stream[stream]["time_types"][livox["time_type"]] += 1
            by_stream[stream]["ports"].update((sport, dport))
            matched += 1

            if matched <= args.udp_dump_limit:
                ts = TIME_TYPES.get(livox["time_type"], "Unknown")
                print(
                    f"[pkt {matched:04d}] {stream:5s} "
                    f"ports={sport:>5}/{dport:<5} time_type={livox['time_type']:>2d} ({ts})"
                    f" data_type={livox['data_type']:>3d} version={livox['version']:>3d}"
                )
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    if matched == 0:
        print("[warn] No matching point/imu packets observed.")
        return 2

    print("\n[summary]")
    for stream in ("point", "imu"):
        if by_stream[stream]["count"] == 0:
            continue
        c = by_stream[stream]["count"]
        types = ", ".join([f"{k}:{v}" for k, v in sorted(by_stream[stream]["time_types"].items())])
        ports = ",".join(map(str, sorted(by_stream[stream]["ports"])))
        print(f"  {stream}: count={c}, ports={ports}, time_type={types}")

        uniq = set(by_stream[stream]["time_types"].keys())
        if uniq == {0}:
            print(f"  {stream}: timestamp likely fallback (NoSync -> host high_resolution_clock::now())")
        elif 0 not in uniq:
            print(f"  {stream}: timestamp from sensor packet sync type(s): {sorted(uniq)}")
        else:
            print(f"  {stream}: mixed time_type values detected: {sorted(uniq)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
