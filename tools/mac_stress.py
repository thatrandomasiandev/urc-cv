#!/usr/bin/env python3
"""
mac_stress — multi-resource stress test for macOS.

Exercises CPU, memory, and optional disk I/O while printing live stats.
Stop anytime with Ctrl+C.

Examples:
  python3 tools/mac_stress.py --yes
  python3 tools/mac_stress.py --yes -d 300 --ram-gb 8 --disk-mb 2048
  python3 tools/mac_stress.py --yes --cpu-only -j 8
"""

from __future__ import annotations

import argparse
import math
import multiprocessing as mp
import os
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field

try:
    import psutil
except ImportError:
    print('Install psutil:  pip install psutil', file=sys.stderr)
    sys.exit(1)


def _cpu_worker(stop: mp.Event, intensity: float) -> None:
    x = 1.0001
    spin = max(1, int(intensity * 50_000))
    while not stop.is_set():
        for _ in range(spin):
            x = math.sin(x) * math.cos(x) + math.sqrt(abs(x) % 1e6 + 1.0)
            x = (x * 1.000001) % 1e9
        if intensity < 0.25:
            time.sleep(0.001)


def _memory_worker(stop: threading.Event, target_bytes: int, chunk_mb: int) -> None:
    chunk = chunk_mb * 1024 * 1024
    blocks: list[bytearray] = []
    allocated = 0
    while not stop.is_set() and allocated < target_bytes:
        need = min(chunk, target_bytes - allocated)
        if need <= 0:
            break
        block = bytearray(need)
        for i in range(0, len(block), 4096):
            block[i] = 0xA5
        blocks.append(block)
        allocated += need
    while not stop.is_set():
        for block in blocks[:: max(1, len(blocks) // 8)]:
            block[0] ^= 0x01
        time.sleep(0.25)


def _disk_worker(stop: threading.Event, path: str, block_kb: int) -> None:
    block = os.urandom(block_kb * 1024)
    with open(path, 'r+b', buffering=0) as f:
        offset = 0
        size = os.path.getsize(path)
        while not stop.is_set():
            f.seek(offset)
            f.write(block)
            f.flush()
            f.seek(offset)
            _ = f.read(len(block))
            offset = (offset + len(block)) % max(size - len(block), 1)


@dataclass
class Sample:
    elapsed_s: float
    cpu_pct: float
    mem_used_gb: float
    mem_pct: float
    swap_gb: float
    load_1: float
    disk_mb_s: float
    temps_c: str


@dataclass
class Monitor:
    stop: threading.Event = field(default_factory=threading.Event)
    interval_s: float = 1.0
    disk_path: str | None = None
    samples: list[Sample] = field(default_factory=list)
    _thread: threading.Thread | None = None
    _prev_disk: tuple[int, float] | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def join(self) -> None:
        if self._thread:
            self._thread.join(timeout=2.0)

    def _disk_rate_mb_s(self) -> float:
        try:
            counters = psutil.disk_io_counters(perdisk=False)
            if counters is None:
                return 0.0
            now = (counters.write_bytes + counters.read_bytes, time.monotonic())
            if self._prev_disk is None:
                self._prev_disk = now
                return 0.0
            prev_bytes, prev_t = self._prev_disk
            dt = now[1] - prev_t
            self._prev_disk = now
            return (now[0] - prev_bytes) / dt / (1024 * 1024) if dt > 0 else 0.0
        except Exception:
            return 0.0

    def _format_temps(self) -> str:
        try:
            sensors = psutil.sensors_temperatures()
        except (AttributeError, OSError):
            return 'n/a'
        if not sensors:
            return 'n/a'
        parts: list[str] = []
        for name, entries in sensors.items():
            for entry in entries[:2]:
                if entry.current is not None:
                    parts.append(f'{entry.label or name}:{entry.current:.0f}°C')
        return ' '.join(parts[:4]) if parts else 'n/a'

    def _run(self) -> None:
        t0 = time.monotonic()
        while not self.stop.wait(self.interval_s):
            vm = psutil.virtual_memory()
            swap = psutil.swap_memory()
            load = os.getloadavg()[0] if hasattr(os, 'getloadavg') else 0.0
            sample = Sample(
                elapsed_s=time.monotonic() - t0,
                cpu_pct=psutil.cpu_percent(interval=None),
                mem_used_gb=vm.used / (1024**3),
                mem_pct=vm.percent,
                swap_gb=swap.used / (1024**3),
                load_1=load,
                disk_mb_s=self._disk_rate_mb_s(),
                temps_c=self._format_temps(),
            )
            self.samples.append(sample)
            print(
                f'[{sample.elapsed_s:6.0f}s] CPU {sample.cpu_pct:5.1f}%  '
                f'RAM {sample.mem_used_gb:5.1f} GB ({sample.mem_pct:4.0f}%)  '
                f'swap {sample.swap_gb:4.2f} GB  load {sample.load_1:4.1f}  '
                f'disk {sample.disk_mb_s:5.1f} MB/s  {sample.temps_c}',
                flush=True,
            )


class StressTest:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self._cpu_stop = mp.Event()
        self._mem_stop = threading.Event()
        self._disk_stop = threading.Event()
        self._cpu_procs: list[mp.Process] = []
        self._disk_file: str | None = None
        self.monitor = Monitor(interval_s=args.interval, disk_path='/')

    def _resolve_ram_bytes(self) -> int:
        if self.args.ram_gb is not None:
            return int(self.args.ram_gb * (1024**3))
        return int(psutil.virtual_memory().total * self.args.ram_fraction)

    def start(self) -> None:
        a = self.args
        ncpu = psutil.cpu_count(logical=True) or 4
        workers = a.jobs if a.jobs else ncpu
        print('=== macOS stress test ===')
        print(f'Host: {os.uname().nodename}  CPUs: {ncpu}  RAM: {psutil.virtual_memory().total / 1e9:.1f} GB')
        print(f'Duration: {a.duration}s  CPU workers: {workers if not a.no_cpu else 0}')
        if not a.no_memory:
            print(f'Memory target: {self._resolve_ram_bytes() / 1e9:.2f} GB')
        if a.disk_mb:
            print(f'Disk stress file: {a.disk_mb} MB')
        print('Press Ctrl+C to stop early.\n')

        if not a.no_cpu:
            ctx = mp.get_context('spawn')
            for _ in range(workers):
                p = ctx.Process(target=_cpu_worker, args=(self._cpu_stop, a.intensity), daemon=True)
                p.start()
                self._cpu_procs.append(p)

        if not a.no_memory:
            threading.Thread(
                target=_memory_worker,
                args=(self._mem_stop, self._resolve_ram_bytes(), a.chunk_mb),
                daemon=True,
            ).start()

        if a.disk_mb:
            fd, path = tempfile.mkstemp(prefix='mac_stress_', suffix='.bin')
            os.close(fd)
            self._disk_file = path
            with open(path, 'wb') as f:
                f.seek(a.disk_mb * 1024 * 1024 - 1)
                f.write(b'\0')
            threading.Thread(
                target=_disk_worker,
                args=(self._disk_stop, path, a.disk_block_kb),
                daemon=True,
            ).start()

        self.monitor.start()

    def stop(self) -> None:
        self._cpu_stop.set()
        self._mem_stop.set()
        self._disk_stop.set()
        self.monitor.stop.set()
        self.monitor.join()
        for p in self._cpu_procs:
            p.join(timeout=3.0)
            if p.is_alive():
                p.terminate()
        if self._disk_file and os.path.exists(self._disk_file):
            os.remove(self._disk_file)

    def run(self) -> int:
        self.start()
        interrupted = False
        try:
            time.sleep(self.args.duration)
        except KeyboardInterrupt:
            interrupted = True
            print('\nInterrupted — shutting down workers...')
        finally:
            self.stop()
            if self.monitor.samples:
                print('\n=== summary ===')
                print(f'Peak CPU: {max(s.cpu_pct for s in self.monitor.samples):.1f}%')
                print(f'Peak RAM: {max(s.mem_pct for s in self.monitor.samples):.1f}%')
        return 130 if interrupted else 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description='Stress-test macOS CPU, memory, and disk.')
    p.add_argument('-d', '--duration', type=float, default=60.0)
    p.add_argument('-j', '--jobs', type=int, default=0)
    p.add_argument('--intensity', type=float, default=1.0)
    p.add_argument('--cpu-only', action='store_true')
    p.add_argument('--no-cpu', action='store_true')
    p.add_argument('--no-memory', action='store_true')
    p.add_argument('--ram-fraction', type=float, default=0.5)
    p.add_argument('--ram-gb', type=float, default=None)
    p.add_argument('--chunk-mb', type=int, default=256)
    p.add_argument('--disk-mb', type=int, default=0)
    p.add_argument('--disk-block-kb', type=int, default=1024)
    p.add_argument('--interval', type=float, default=1.0)
    p.add_argument('--yes', action='store_true')
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    args.intensity = max(0.05, min(1.0, args.intensity))
    args.ram_fraction = max(0.05, min(0.95, args.ram_fraction))
    if args.cpu_only:
        args.no_memory = True
        args.disk_mb = 0
    if not args.yes:
        try:
            if input('Continue? [y/N] ').strip().lower() not in ('y', 'yes'):
                return 0
        except (EOFError, KeyboardInterrupt):
            return 130
    mp.set_start_method('spawn', force=True)
    return StressTest(args).run()


if __name__ == '__main__':
    raise SystemExit(main())
