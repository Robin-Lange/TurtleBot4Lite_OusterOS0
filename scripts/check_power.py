#!/usr/bin/env python3
"""Jetson Orin Nano Power and Hardware Health Monitor."""

import argparse
import re
import subprocess
import sys
import time

def get_nvpmodel():
    try:
        res = subprocess.run(["nvpmodel", "-q"], capture_output=True, text=True, check=False)
        for line in res.stdout.splitlines():
            if "NVPM" in line or "Mode" in line:
                return line.strip()
    except Exception:
        pass
    return "Unknown"

def parse_line(line):
    vdd_in = re.search(r'VDD_IN\s+(\d+)mW', line)
    vdd_cpu = re.search(r'VDD_CPU_GPU_CV\s+(\d+)mW', line)
    vdd_soc = re.search(r'VDD_SOC\s+(\d+)mW', line)
    ram = re.search(r'RAM\s+([\d/]+MB)', line)
    swap = re.search(r'SWAP\s+([\d/]+MB)', line)
    cpu_temp = re.search(r'cpu@([\d.]+C)', line)
    gpu_temp = re.search(r'gpu@([\d.]+C)', line)
    tj_temp = re.search(r'tj@([\d.]+C)', line)
    
    in_mw = int(vdd_in.group(1)) if vdd_in else 0
    cpu_mw = int(vdd_cpu.group(1)) if vdd_cpu else 0
    soc_mw = int(vdd_soc.group(1)) if vdd_soc else 0
    
    return {
        "in_w": in_mw / 1000.0,
        "in_mw": in_mw,
        "cpu_w": cpu_mw / 1000.0,
        "cpu_mw": cpu_mw,
        "soc_w": soc_mw / 1000.0,
        "soc_mw": soc_mw,
        "ram": ram.group(1) if ram else "N/A",
        "swap": swap.group(1) if swap else "N/A",
        "cpu_temp": cpu_temp.group(1) if cpu_temp else "N/A",
        "gpu_temp": gpu_temp.group(1) if gpu_temp else "N/A",
        "tj_temp": tj_temp.group(1) if tj_temp else "N/A",
    }

def print_summary(data, power_mode):
    print("=" * 60)
    print("       NVIDIA Jetson Orin Nano Power & Health Monitor       ")
    print("=" * 60)
    print(f"Power Mode     : {power_mode}")
    print(f"Total Power In : {data['in_w']:.2f} W  ({data['in_mw']} mW)")
    print(f"CPU/GPU/CV     : {data['cpu_w']:.2f} W  ({data['cpu_mw']} mW)")
    print(f"SOC Power      : {data['soc_w']:.2f} W  ({data['soc_mw']} mW)")
    print("-" * 60)
    print(f"Temperatures   : CPU: {data['cpu_temp']} | GPU: {data['gpu_temp']} | TJunction: {data['tj_temp']}")
    print(f"Memory Usage   : RAM: {data['ram']} | SWAP: {data['swap']}")
    print("=" * 60)

def main():
    parser = argparse.ArgumentParser(description="Check Jetson power consumption and thermals.")
    parser.add_argument("-c", "--continuous", action="store_true", help="Monitor continuously (1s interval)")
    args = parser.parse_args()

    power_mode = get_nvpmodel()

    if not args.continuous:
        proc = subprocess.Popen(["tegrastats", "--interval", "500"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            line = proc.stdout.readline()
        finally:
            proc.terminate()
        data = parse_line(line)
        print_summary(data, power_mode)
        print("Tip: Run with -c or --continuous for live streaming.")
    else:
        print(f"Monitoring power continuously (Press Ctrl+C to stop)... Power Mode: {power_mode}")
        proc = subprocess.Popen(["tegrastats", "--interval", "1000"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            while True:
                line = proc.stdout.readline()
                if not line:
                    break
                data = parse_line(line)
                timestamp = time.strftime("%H:%M:%S")
                print(f"[{timestamp}] Power: {data['in_w']:.2f}W ({data['in_mw']}mW) | CPU: {data['cpu_temp']} | GPU: {data['gpu_temp']} | RAM: {data['ram']}")
        except KeyboardInterrupt:
            print("\nStopped monitor.")
        finally:
            proc.terminate()

if __name__ == "__main__":
    main()
