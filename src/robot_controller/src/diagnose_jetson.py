#!/usr/bin/env python3
"""
Jetson Nano Resource Monitor
=============================
Monitors CPU, RAM, network, temperature on Jetson Nano in real-time.

IMPORTANT: This script must be run ON THE JETSON, not on the master PC.

Usage:
    # On Jetson:
    python3 diagnose_jetson.py

    # Or via ROS:
    rosrun robot_controller diagnose_jetson.py
"""

import psutil
import time
import subprocess
import os
from datetime import datetime


class JetsonMonitor:
    def __init__(self):
        self.interval = 1.0  # Update every 1 second

        # Check if running on Jetson
        self.is_jetson = self.check_jetson()

        # Network interface (usually wlan0 for WiFi on Jetson)
        self.net_interface = self.detect_network_interface()

        # Previous network stats for rate calculation
        self.prev_net_io = psutil.net_io_counters()
        self.prev_time = time.time()

        print("\n" + "=" * 80)
        print("  JETSON NANO RESOURCE MONITOR")
        print("=" * 80)
        if self.is_jetson:
            print("  Status: Running on Jetson Nano ✓")
        else:
            print("  WARNING: Not detected as Jetson - some features may not work")
        print("  Network Interface: " + self.net_interface)
        print("=" * 80 + "\n")

    def check_jetson(self):
        """Check if running on Jetson Nano"""
        try:
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read()
                return 'jetson' in model.lower()
        except:
            return False

    def detect_network_interface(self):
        """Detect active network interface"""
        interfaces = psutil.net_if_stats()

        # Prefer WiFi (wlan0), then Ethernet (eth0)
        if 'wlan0' in interfaces and interfaces['wlan0'].isup:
            return 'wlan0'
        elif 'eth0' in interfaces and interfaces['eth0'].isup:
            return 'eth0'
        else:
            # Return first active interface
            for iface, stats in interfaces.items():
                if stats.isup and iface != 'lo':
                    return iface
        return 'unknown'

    def get_cpu_percent_per_core(self):
        """Get CPU usage per core"""
        return psutil.cpu_percent(interval=0.1, percpu=True)

    def get_memory_info(self):
        """Get memory usage"""
        mem = psutil.virtual_memory()
        return {
            'total_mb': mem.total / 1024 / 1024,
            'used_mb': mem.used / 1024 / 1024,
            'available_mb': mem.available / 1024 / 1024,
            'percent': mem.percent
        }

    def get_network_bandwidth(self):
        """Calculate network TX/RX rate in KB/s"""
        current_net_io = psutil.net_io_counters()
        current_time = time.time()

        dt = current_time - self.prev_time

        if dt > 0:
            tx_rate = (current_net_io.bytes_sent - self.prev_net_io.bytes_sent) / dt / 1024  # KB/s
            rx_rate = (current_net_io.bytes_recv - self.prev_net_io.bytes_recv) / dt / 1024  # KB/s
        else:
            tx_rate = 0
            rx_rate = 0

        self.prev_net_io = current_net_io
        self.prev_time = current_time

        return tx_rate, rx_rate

    def get_temperature(self):
        """Get Jetson temperature (thermal zones)"""
        temps = []

        if self.is_jetson:
            # Jetson specific thermal zones
            thermal_zones = [
                '/sys/devices/virtual/thermal/thermal_zone0/temp',  # CPU
                '/sys/devices/virtual/thermal/thermal_zone1/temp',  # GPU
            ]

            for zone in thermal_zones:
                try:
                    with open(zone, 'r') as f:
                        temp = int(f.read().strip()) / 1000.0  # Convert mC to C
                        temps.append(temp)
                except:
                    pass
        else:
            # Fallback to psutil for non-Jetson
            try:
                sensors = psutil.sensors_temperatures()
                for name, entries in sensors.items():
                    for entry in entries:
                        temps.append(entry.current)
            except:
                pass

        return temps

    def get_top_ros_processes(self, n=5):
        """Get top N ROS processes by CPU usage"""
        ros_processes = []

        for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent']):
            try:
                pinfo = proc.info
                # Filter ROS-related processes
                if any(keyword in pinfo['name'].lower() for keyword in ['ros', 'python', 'node']):
                    ros_processes.append(pinfo)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

        # Sort by CPU usage
        ros_processes.sort(key=lambda x: x['cpu_percent'], reverse=True)

        return ros_processes[:n]

    def clear_screen(self):
        """Clear terminal screen"""
        os.system('clear' if os.name == 'posix' else 'cls')

    def print_stats(self):
        """Print all statistics"""
        self.clear_screen()

        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        print("=" * 80)
        print(f"  JETSON NANO DIAGNOSTICS - {timestamp}")
        print("=" * 80)

        # CPU
        print("\n[CPU USAGE]")
        cpu_percents = self.get_cpu_percent_per_core()
        for i, cpu in enumerate(cpu_percents):
            bar_length = int(cpu / 2)  # Scale to 50 chars max
            bar = '█' * bar_length + '░' * (50 - bar_length)
            print(f"  Core {i}: {bar} {cpu:5.1f}%")

        avg_cpu = sum(cpu_percents) / len(cpu_percents)
        print(f"  Average: {avg_cpu:5.1f}%")

        # Memory
        print("\n[MEMORY]")
        mem = self.get_memory_info()
        bar_length = int(mem['percent'] / 2)
        bar = '█' * bar_length + '░' * (50 - bar_length)
        print(f"  Usage:   {bar} {mem['percent']:5.1f}%")
        print(f"  Used:    {mem['used_mb']:7.1f} MB / {mem['total_mb']:.1f} MB")
        print(f"  Free:    {mem['available_mb']:7.1f} MB")

        # Temperature
        temps = self.get_temperature()
        if temps:
            print("\n[TEMPERATURE]")
            for i, temp in enumerate(temps):
                zone_name = "CPU" if i == 0 else f"Zone{i}"
                status = "⚠️  HOT" if temp > 70 else "OK"
                print(f"  {zone_name}: {temp:5.1f}°C  [{status}]")

        # Network
        print("\n[NETWORK]")
        tx_rate, rx_rate = self.get_network_bandwidth()
        print(f"  TX (Upload):   {tx_rate:8.2f} KB/s")
        print(f"  RX (Download): {rx_rate:8.2f} KB/s")
        print(f"  Total:         {tx_rate + rx_rate:8.2f} KB/s")

        # Top ROS processes
        print("\n[TOP ROS PROCESSES BY CPU]")
        top_procs = self.get_top_ros_processes(5)
        if top_procs:
            print(f"  {'PID':<8} {'NAME':<25} {'CPU%':<8} {'MEM%':<8}")
            print("  " + "-" * 55)
            for proc in top_procs:
                print(f"  {proc['pid']:<8} {proc['name']:<25} {proc['cpu_percent']:<8.1f} {proc['memory_percent']:<8.1f}")
        else:
            print("  No ROS processes detected")

        # Warnings
        print("\n[SYSTEM STATUS]")
        warnings = []

        if avg_cpu > 80:
            warnings.append("⚠️  CPU usage > 80% - System overload!")
        if mem['percent'] > 80:
            warnings.append("⚠️  Memory usage > 80% - Risk of OOM!")
        if temps and any(t > 75 for t in temps):
            warnings.append("⚠️  Temperature > 75°C - Thermal throttling risk!")
        if tx_rate + rx_rate > 1000:  # > 1 MB/s
            warnings.append("⚠️  Network bandwidth > 1 MB/s - High traffic!")

        if warnings:
            for warning in warnings:
                print(f"  {warning}")
        else:
            print("  ✓ All systems normal")

        print("\n" + "=" * 80)
        print("  Press Ctrl+C to exit")
        print("=" * 80)

    def run(self):
        """Main monitoring loop"""
        try:
            while True:
                self.print_stats()
                time.sleep(self.interval)
        except KeyboardInterrupt:
            print("\n\nMonitoring stopped.")


if __name__ == "__main__":
    monitor = JetsonMonitor()
    monitor.run()
