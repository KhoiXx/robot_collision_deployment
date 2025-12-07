#!/usr/bin/env python3
"""
Robot Specifications Validator
================================
Cross-checks all robot parameters across different configuration files.

Usage:
    python3 validate_robot_specs.py
"""

import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from settings import ROBOT_WHEEL_DISTANCE, ROBOT_RADIUS, ACTION_BOUND


def parse_urdf_wheel_separation(urdf_path):
    """Extract wheel separation from URDF joint positions"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Find left and right wheel joints
    left_joint = root.find(".//joint[@name='left_wheel_jl']/origin")
    right_joint = root.find(".//joint[@name='right_wheel_jl']/origin")

    if left_joint is not None and right_joint is not None:
        left_xyz = left_joint.get('xyz').split()
        right_xyz = right_joint.get('xyz').split()

        # Y-coordinate contains wheel position
        left_y = float(left_xyz[1])
        right_y = float(right_xyz[1])

        # Wheel separation = distance between centers
        separation = abs(left_y - right_y)
        return separation, left_y, right_y

    return None, None, None


def parse_gazebo_params(gazebo_path):
    """Extract wheel parameters from Gazebo plugin"""
    tree = ET.parse(gazebo_path)
    root = tree.getroot()

    # Find differential drive plugin
    wheel_sep = root.find(".//wheelSeparation")
    wheel_dia = root.find(".//wheelDiameter")

    separation = float(wheel_sep.text) if wheel_sep is not None else None
    diameter = float(wheel_dia.text) if wheel_dia is not None else None
    radius = diameter / 2.0 if diameter else None

    return separation, radius, diameter


def parse_env_file(env_path):
    """Extract parameters from .env file"""
    params = {}

    with open(env_path, 'r') as f:
        for line in f:
            line = line.strip()
            if '=' in line and not line.startswith('#'):
                key, value = line.split('=', 1)
                key = key.strip()
                value = value.strip()

                if key in ['ROBOT_WHEEL_DISTANCE', 'ROBOT_RADIUS']:
                    params[key] = float(value)
                elif key == 'ACTION_BOUND':
                    # Parse ACTION_BOUND
                    params[key] = value

    return params


def parse_arduino_code(arduino_path):
    """Extract parameters from Arduino code"""
    params = {}

    with open(arduino_path, 'r') as f:
        content = f.read()

        # Find ROBOT_WHEEL_DISTANCE
        match = re.search(r'#define\s+ROBOT_WHEEL_DISTANCE\s+([\d.]+)', content)
        if match:
            params['ROBOT_WHEEL_DISTANCE'] = float(match.group(1))

        # Find wheel radius in constructor
        # MotorController leftWheel(clk, dt, in1, in2, en, PPR, radius, Kp, Ki, Kd, sample, maxSpeed, dir)
        # Extract all numeric parameters between parentheses
        match = re.search(r'MotorController\s+leftWheel\s*\(\s*([^)]+)\)', content)
        if match:
            params_str = match.group(1)
            # Split by comma, get 7th parameter (index 6) = wheelRadius
            params_list = [p.strip() for p in params_str.split(',')]
            if len(params_list) >= 7:
                try:
                    # Parameter 6 (0-indexed) is wheelRadius
                    params['WHEEL_RADIUS'] = float(params_list[6])
                except (ValueError, IndexError):
                    pass

    return params


def print_section(title):
    """Print section header"""
    print("\n" + "=" * 80)
    print(f"  {title}")
    print("=" * 80)


def print_param(name, value, unit="", status=""):
    """Print parameter with formatting"""
    if status:
        print(f"  {name:30s} {value:>12} {unit:6s} {status}")
    else:
        print(f"  {name:30s} {value:>12} {unit}")


def main():
    # Paths - absolute from script location
    script_path = Path(__file__).resolve()
    deployment_root = script_path.parent.parent.parent.parent  # Go up to deployment/

    urdf_path = deployment_root / "src" / "robot_description" / "urdf" / "robot_description.urdf"
    gazebo_path = deployment_root / "src" / "robot_description" / "urdf" / "robot_description.gazebo.xml"
    env_path = deployment_root / "src" / "robot_controller" / "src" / ".env"
    arduino_path = deployment_root / "src" / "esp32" / "controller_copy" / "controller_copy.ino"

    print("\n" + "█" * 80)
    print("█" + " " * 78 + "█")
    print("█" + "  ROBOT SPECIFICATIONS VALIDATION REPORT".center(78) + "█")
    print("█" + " " * 78 + "█")
    print("█" * 80)

    # ========================================================================
    # URDF Analysis
    # ========================================================================
    print_section("1. URDF (Physical Model)")

    urdf_sep, left_y, right_y = parse_urdf_wheel_separation(urdf_path)

    if urdf_sep:
        print_param("Left wheel Y position", f"{left_y:.6f}", "m")
        print_param("Right wheel Y position", f"{right_y:.6f}", "m")
        print_param("Wheel Separation (calculated)", f"{urdf_sep:.4f}", "m")
        print_param("", f"{urdf_sep*100:.2f}", "cm")

    # ========================================================================
    # Gazebo Analysis
    # ========================================================================
    print_section("2. GAZEBO (Simulation)")

    gazebo_sep, gazebo_radius, gazebo_dia = parse_gazebo_params(gazebo_path)

    if gazebo_sep:
        print_param("Wheel Separation", f"{gazebo_sep:.4f}", "m")
        print_param("", f"{gazebo_sep*100:.2f}", "cm")
    if gazebo_dia:
        print_param("Wheel Diameter", f"{gazebo_dia:.4f}", "m")
        print_param("Wheel Radius", f"{gazebo_radius:.4f}", "m")
        print_param("", f"{gazebo_radius*100:.2f}", "cm")

    # ========================================================================
    # .env Analysis
    # ========================================================================
    print_section("3. .ENV (Python Training/Deployment)")

    env_params = parse_env_file(env_path)

    if 'ROBOT_WHEEL_DISTANCE' in env_params:
        dist = env_params['ROBOT_WHEEL_DISTANCE']
        print_param("ROBOT_WHEEL_DISTANCE", f"{dist:.4f}", "m")
        print_param("", f"{dist*100:.2f}", "cm")

    if 'ROBOT_RADIUS' in env_params:
        radius = env_params['ROBOT_RADIUS']
        print_param("ROBOT_RADIUS", f"{radius:.4f}", "m")
        print_param("", f"{radius*100:.2f}", "cm")

    if 'ACTION_BOUND' in env_params:
        print_param("ACTION_BOUND", env_params['ACTION_BOUND'], "")
        # Parse and display
        try:
            import ast
            bounds = ast.literal_eval(env_params['ACTION_BOUND'])
            max_linear = bounds[1][0]
            max_angular = abs(bounds[0][1]) if abs(bounds[0][1]) > abs(bounds[1][1]) else abs(bounds[1][1])
            print_param("  Max Linear Velocity", f"{max_linear:.2f}", "m/s")
            print_param("  Max Angular Velocity", f"{max_angular:.2f}", "rad/s")
        except:
            pass

    # ========================================================================
    # Arduino Analysis
    # ========================================================================
    print_section("4. ARDUINO (Hardware Controller)")

    arduino_params = parse_arduino_code(arduino_path)

    if 'ROBOT_WHEEL_DISTANCE' in arduino_params:
        dist = arduino_params['ROBOT_WHEEL_DISTANCE']
        print_param("ROBOT_WHEEL_DISTANCE", f"{dist:.4f}", "m")
        print_param("", f"{dist*100:.2f}", "cm")

    if 'WHEEL_RADIUS' in arduino_params:
        radius = arduino_params['WHEEL_RADIUS']
        print_param("Wheel Radius", f"{radius:.4f}", "m")
        print_param("", f"{radius*100:.2f}", "cm")

    # ========================================================================
    # Python Runtime (settings.py)
    # ========================================================================
    print_section("5. PYTHON RUNTIME (settings.py)")

    print_param("ROBOT_WHEEL_DISTANCE", f"{ROBOT_WHEEL_DISTANCE:.4f}", "m")
    print_param("", f"{ROBOT_WHEEL_DISTANCE*100:.2f}", "cm")
    print_param("ROBOT_RADIUS", f"{ROBOT_RADIUS:.4f}", "m")
    print_param("", f"{ROBOT_RADIUS*100:.2f}", "cm")
    print_param("ACTION_BOUND", str(ACTION_BOUND), "")

    # ========================================================================
    # CONSISTENCY CHECK
    # ========================================================================
    print_section("6. CONSISTENCY CHECK")

    # Expected values
    EXPECTED_WHEEL_SEP = 0.204  # 20.4 cm
    EXPECTED_WHEEL_RADIUS = 0.0342  # 3.42 cm
    EXPECTED_MAX_VEL = 0.3  # 0.3 m/s

    TOLERANCE = 0.001  # 1mm tolerance

    all_pass = True

    # Check wheel separation
    print("\n  Wheel Separation (20.4 cm = 0.204 m):")

    if urdf_sep:
        match = abs(urdf_sep - EXPECTED_WHEEL_SEP) < TOLERANCE
        status = "✓ PASS" if match else f"✗ FAIL (diff: {(urdf_sep - EXPECTED_WHEEL_SEP)*100:.2f} cm)"
        print_param("  URDF", f"{urdf_sep*100:.2f}", "cm", status)
        all_pass = all_pass and match

    if gazebo_sep:
        match = abs(gazebo_sep - EXPECTED_WHEEL_SEP) < TOLERANCE
        status = "✓ PASS" if match else f"✗ FAIL (diff: {(gazebo_sep - EXPECTED_WHEEL_SEP)*100:.2f} cm)"
        print_param("  Gazebo", f"{gazebo_sep*100:.2f}", "cm", status)
        all_pass = all_pass and match

    if 'ROBOT_WHEEL_DISTANCE' in env_params:
        dist = env_params['ROBOT_WHEEL_DISTANCE']
        match = abs(dist - EXPECTED_WHEEL_SEP) < TOLERANCE
        status = "✓ PASS" if match else f"✗ FAIL (diff: {(dist - EXPECTED_WHEEL_SEP)*100:.2f} cm)"
        print_param("  .env", f"{dist*100:.2f}", "cm", status)
        all_pass = all_pass and match

    if 'ROBOT_WHEEL_DISTANCE' in arduino_params:
        dist = arduino_params['ROBOT_WHEEL_DISTANCE']
        match = abs(dist - EXPECTED_WHEEL_SEP) < TOLERANCE
        status = "✓ PASS" if match else f"✗ FAIL (diff: {(dist - EXPECTED_WHEEL_SEP)*100:.2f} cm)"
        print_param("  Arduino", f"{dist*100:.2f}", "cm", status)
        all_pass = all_pass and match

    match = abs(ROBOT_WHEEL_DISTANCE - EXPECTED_WHEEL_SEP) < TOLERANCE
    status = "✓ PASS" if match else f"✗ FAIL (diff: {(ROBOT_WHEEL_DISTANCE - EXPECTED_WHEEL_SEP)*100:.2f} cm)"
    print_param("  Python", f"{ROBOT_WHEEL_DISTANCE*100:.2f}", "cm", status)
    all_pass = all_pass and match

    # Check wheel radius
    print("\n  Wheel Radius (3.42 cm = 0.0342 m):")

    if gazebo_radius:
        match = abs(gazebo_radius - EXPECTED_WHEEL_RADIUS) < TOLERANCE
        status = "✓ PASS" if match else f"✗ FAIL (diff: {(gazebo_radius - EXPECTED_WHEEL_RADIUS)*100:.2f} cm)"
        print_param("  Gazebo", f"{gazebo_radius*100:.2f}", "cm", status)
        all_pass = all_pass and match

    if 'WHEEL_RADIUS' in arduino_params:
        radius = arduino_params['WHEEL_RADIUS']
        match = abs(radius - EXPECTED_WHEEL_RADIUS) < TOLERANCE
        status = "✓ PASS" if match else f"✗ FAIL (diff: {(radius - EXPECTED_WHEEL_RADIUS)*100:.2f} cm)"
        print_param("  Arduino", f"{radius*100:.2f}", "cm", status)
        all_pass = all_pass and match

    # Check max velocity
    print("\n  Max Linear Velocity (0.3 m/s):")

    try:
        max_linear = ACTION_BOUND[1][0]
        match = abs(max_linear - EXPECTED_MAX_VEL) < 0.01
        status = "✓ PASS" if match else f"✗ FAIL (diff: {(max_linear - EXPECTED_MAX_VEL):.2f} m/s)"
        print_param("  ACTION_BOUND", f"{max_linear:.2f}", "m/s", status)
        all_pass = all_pass and match
    except:
        pass

    # ========================================================================
    # FINAL RESULT
    # ========================================================================
    print("\n" + "=" * 80)
    if all_pass:
        print("  ✓ ALL PARAMETERS SYNCHRONIZED")
    else:
        print("  ✗ INCONSISTENCIES DETECTED - Please review above")
    print("=" * 80 + "\n")

    return 0 if all_pass else 1


if __name__ == "__main__":
    exit(main())
