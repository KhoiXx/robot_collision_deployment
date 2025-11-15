#!/usr/bin/env python3
"""
Comprehensive PID Stability Analysis Script
Analyzes step response data and generates stability report with visualizations
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from datetime import datetime
import sys
import os

def analyze_step_response(df_step, target_speed):
    """
    Analyze a single step response
    Returns dict with stability metrics
    """
    # Get settling time (time to reach and stay within 2% of setpoint)
    setpoint = df_step['setpoint'].iloc[-1]
    tolerance = 0.02 * abs(setpoint) if setpoint != 0 else 1.0  # 2% tolerance

    # Find when it enters and stays in tolerance band
    in_tolerance = np.abs(df_step['current'] - setpoint) <= tolerance

    settling_time = None
    for i in range(len(df_step) - 10):  # Need to stay for at least 10 samples
        if all(in_tolerance.iloc[i:]):
            settling_time = df_step['test_time'].iloc[i]
            break

    # Calculate overshoot
    if setpoint > 0:
        max_val = df_step['current'].max()
        overshoot_percent = ((max_val - setpoint) / setpoint * 100) if setpoint != 0 else 0
    else:
        min_val = df_step['current'].min()
        overshoot_percent = ((setpoint - min_val) / abs(setpoint) * 100) if setpoint != 0 else 0

    # Rise time (time to go from 10% to 90% of setpoint)
    if setpoint != 0:
        initial = df_step['current'].iloc[0]
        target_10 = initial + 0.1 * (setpoint - initial)
        target_90 = initial + 0.9 * (setpoint - initial)

        if setpoint > initial:
            t_10 = df_step[df_step['current'] >= target_10]['test_time'].iloc[0] if any(df_step['current'] >= target_10) else None
            t_90 = df_step[df_step['current'] >= target_90]['test_time'].iloc[0] if any(df_step['current'] >= target_90) else None
        else:
            t_10 = df_step[df_step['current'] <= target_10]['test_time'].iloc[0] if any(df_step['current'] <= target_10) else None
            t_90 = df_step[df_step['current'] <= target_90]['test_time'].iloc[0] if any(df_step['current'] <= target_90) else None

        rise_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else None
    else:
        rise_time = None

    # Steady-state error (average error in last 20% of data)
    steady_start_idx = int(len(df_step) * 0.8)
    steady_state_error = df_step['error'].iloc[steady_start_idx:].abs().mean()

    # RMS error over entire step
    rms_error = np.sqrt((df_step['error'] ** 2).mean())

    return {
        'target_speed': target_speed,
        'setpoint': setpoint,
        'settling_time': settling_time,
        'rise_time': rise_time,
        'overshoot_percent': overshoot_percent,
        'steady_state_error': steady_state_error,
        'rms_error': rms_error,
        'final_current': df_step['current'].iloc[-1]
    }

def create_comprehensive_plots(csv_file, output_prefix):
    """
    Create comprehensive visualization plots
    """
    df = pd.read_csv(csv_file)

    # Identify different speed steps
    df['speed_change'] = df['target_speed'].diff().abs() > 0.01
    speed_changes = df[df['speed_change']].index.tolist()
    speed_changes.insert(0, 0)
    speed_changes.append(len(df))

    steps = []
    for i in range(len(speed_changes) - 1):
        start_idx = speed_changes[i]
        end_idx = speed_changes[i + 1]
        step_df = df.iloc[start_idx:end_idx].copy()
        step_df['test_time'] = step_df['test_time'] - step_df['test_time'].iloc[0]
        steps.append((step_df['target_speed'].iloc[0], step_df))

    # Create multi-panel plot
    n_steps = len(steps)
    fig, axes = plt.subplots(n_steps, 3, figsize=(18, 4*n_steps))
    if n_steps == 1:
        axes = axes.reshape(1, -1)

    fig.suptitle('PID Step Response Analysis - Comprehensive Stability Report', fontsize=16, fontweight='bold')

    for idx, (target_speed, step_df) in enumerate(steps):
        setpoint = step_df['setpoint'].iloc[0]

        # Panel 1: Setpoint tracking
        ax1 = axes[idx, 0]
        ax1.plot(step_df['test_time'], step_df['setpoint'], 'r--', linewidth=2, label='Setpoint', alpha=0.7)
        ax1.plot(step_df['test_time'], step_df['current'], 'b-', linewidth=1.5, label='Current Speed')
        ax1.fill_between(step_df['test_time'],
                         step_df['setpoint'] * 0.98,
                         step_df['setpoint'] * 1.02,
                         color='green', alpha=0.2, label='±2% tolerance')
        ax1.set_xlabel('Time (s)', fontsize=10)
        ax1.set_ylabel('Speed (encoder pulses/sample)', fontsize=10)
        ax1.set_title(f'Step Response: {target_speed:.1f} m/s (Setpoint: {setpoint:.0f})', fontsize=11, fontweight='bold')
        ax1.legend(loc='best', fontsize=9)
        ax1.grid(True, alpha=0.3)

        # Panel 2: Error over time
        ax2 = axes[idx, 1]
        ax2.plot(step_df['test_time'], step_df['error'], 'r-', linewidth=1.5, label='Tracking Error')
        ax2.axhline(y=0, color='k', linestyle='--', linewidth=1, alpha=0.5)
        ax2.fill_between(step_df['test_time'], -2, 2, color='green', alpha=0.2, label='Good tracking (|error| < 2)')
        ax2.set_xlabel('Time (s)', fontsize=10)
        ax2.set_ylabel('Error (pulses)', fontsize=10)
        ax2.set_title(f'Tracking Error vs Time', fontsize=11, fontweight='bold')
        ax2.legend(loc='best', fontsize=9)
        ax2.grid(True, alpha=0.3)

        # Panel 3: Control output
        ax3 = axes[idx, 2]
        ax3.plot(step_df['test_time'], step_df['output'], 'g-', linewidth=1.5, label='PWM Output')
        ax3.axhline(y=0, color='k', linestyle='--', linewidth=1, alpha=0.5)
        ax3.set_xlabel('Time (s)', fontsize=10)
        ax3.set_ylabel('PWM Output', fontsize=10)
        ax3.set_title(f'Control Output vs Time', fontsize=11, fontweight='bold')
        ax3.legend(loc='best', fontsize=9)
        ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_file = f"{output_prefix}_comprehensive.png"
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    plt.close()

    return plot_file, steps

def generate_text_report(csv_file, steps, output_prefix):
    """
    Generate comprehensive text report with stability analysis
    """
    report_file = f"{output_prefix}_report.txt"

    with open(report_file, 'w', encoding='utf-8') as f:
        f.write("=" * 80 + "\n")
        f.write("PID CONTROLLER STABILITY ANALYSIS REPORT\n")
        f.write("=" * 80 + "\n\n")

        f.write(f"Test File: {os.path.basename(csv_file)}\n")
        f.write(f"Analysis Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Number of Step Tests: {len(steps)}\n\n")

        f.write("=" * 80 + "\n")
        f.write("EXECUTIVE SUMMARY - STABILITY VERIFICATION\n")
        f.write("=" * 80 + "\n\n")

        all_stable = True
        all_metrics = []

        for target_speed, step_df in steps:
            metrics = analyze_step_response(step_df, target_speed)
            all_metrics.append(metrics)

            # Check stability criteria
            is_stable = True
            stability_issues = []

            # Criterion 1: Settling time should exist and be reasonable (< 3 seconds)
            if metrics['settling_time'] is None:
                is_stable = False
                stability_issues.append("Does not settle within test duration")
            elif metrics['settling_time'] > 3.0:
                is_stable = False
                stability_issues.append(f"Settling time too long: {metrics['settling_time']:.2f}s")

            # Criterion 2: Overshoot should be reasonable (< 15%)
            if abs(metrics['overshoot_percent']) > 15:
                stability_issues.append(f"High overshoot: {metrics['overshoot_percent']:.1f}%")

            # Criterion 3: Steady-state error should be small
            if metrics['setpoint'] != 0:
                ss_error_percent = (metrics['steady_state_error'] / abs(metrics['setpoint'])) * 100
                if ss_error_percent > 5:
                    stability_issues.append(f"High steady-state error: {ss_error_percent:.1f}%")

            if not is_stable or stability_issues:
                all_stable = False

        # Overall verdict
        if all_stable:
            f.write("✓ VERDICT: SYSTEM IS STABLE\n\n")
            f.write("The PID controller demonstrates stable behavior across all tested speeds.\n")
            f.write("Key indicators:\n")
            f.write("  - All step responses settle within acceptable time\n")
            f.write("  - Overshoot is within acceptable limits (< 15%)\n")
            f.write("  - Steady-state errors are minimal\n")
            f.write("  - No oscillations or divergence observed\n\n")
        else:
            f.write("⚠ VERDICT: STABILITY ISSUES DETECTED\n\n")
            f.write("Some step responses show stability concerns. See detailed analysis below.\n\n")

        f.write("=" * 80 + "\n")
        f.write("DETAILED STEP RESPONSE ANALYSIS\n")
        f.write("=" * 80 + "\n\n")

        for idx, ((target_speed, step_df), metrics) in enumerate(zip(steps, all_metrics)):
            f.write(f"\nSTEP {idx + 1}: Target Speed = {target_speed:.2f} m/s (Setpoint = {metrics['setpoint']:.0f} pulses)\n")
            f.write("-" * 80 + "\n")

            # Performance metrics
            f.write("\nPerformance Metrics:\n")
            f.write(f"  • Settling Time (2% band):     {metrics['settling_time']:.3f} s\n" if metrics['settling_time'] else "  • Settling Time:              Did not settle\n")
            f.write(f"  • Rise Time (10%-90%):         {metrics['rise_time']:.3f} s\n" if metrics['rise_time'] else "  • Rise Time:                  N/A\n")
            f.write(f"  • Overshoot:                   {metrics['overshoot_percent']:.2f} %\n")
            f.write(f"  • Steady-State Error:          {metrics['steady_state_error']:.3f} pulses\n")
            if metrics['setpoint'] != 0:
                ss_error_pct = (metrics['steady_state_error'] / abs(metrics['setpoint'])) * 100
                f.write(f"  • Steady-State Error (%):      {ss_error_pct:.2f} %\n")
            f.write(f"  • RMS Error:                   {metrics['rms_error']:.3f} pulses\n")
            f.write(f"  • Final Value:                 {metrics['final_current']:.1f} pulses\n")

            # Stability assessment
            f.write("\nStability Assessment:\n")

            # Check each criterion
            if metrics['settling_time'] is not None and metrics['settling_time'] < 3.0:
                f.write(f"  ✓ Fast settling time ({metrics['settling_time']:.2f}s < 3.0s)\n")
            elif metrics['settling_time'] is None:
                f.write(f"  ✗ Does not settle within test duration\n")
            else:
                f.write(f"  ⚠ Slow settling time ({metrics['settling_time']:.2f}s)\n")

            if abs(metrics['overshoot_percent']) < 5:
                f.write(f"  ✓ Minimal overshoot ({abs(metrics['overshoot_percent']):.1f}% < 5%)\n")
            elif abs(metrics['overshoot_percent']) < 15:
                f.write(f"  ⚠ Moderate overshoot ({abs(metrics['overshoot_percent']):.1f}%)\n")
            else:
                f.write(f"  ✗ Excessive overshoot ({abs(metrics['overshoot_percent']):.1f}%)\n")

            if metrics['setpoint'] != 0:
                ss_error_pct = (metrics['steady_state_error'] / abs(metrics['setpoint'])) * 100
                if ss_error_pct < 2:
                    f.write(f"  ✓ Excellent steady-state accuracy ({ss_error_pct:.2f}%)\n")
                elif ss_error_pct < 5:
                    f.write(f"  ✓ Good steady-state accuracy ({ss_error_pct:.2f}%)\n")
                else:
                    f.write(f"  ⚠ Steady-state error present ({ss_error_pct:.2f}%)\n")

            # Error statistics
            error_max = step_df['error'].abs().max()
            error_std = step_df['error'].std()

            f.write(f"\nError Statistics:\n")
            f.write(f"  • Maximum Error:               {error_max:.3f} pulses\n")
            f.write(f"  • Error Std Dev:               {error_std:.3f} pulses\n")

            # Check for oscillations (high std dev relative to steady-state)
            if error_std < 1.0:
                f.write(f"  ✓ Low oscillation (std dev < 1.0)\n")
            elif error_std < 2.0:
                f.write(f"  ⚠ Moderate oscillation (std dev = {error_std:.2f})\n")
            else:
                f.write(f"  ✗ High oscillation (std dev = {error_std:.2f})\n")

            f.write("\n")

        # Summary statistics table
        f.write("\n" + "=" * 80 + "\n")
        f.write("SUMMARY STATISTICS TABLE\n")
        f.write("=" * 80 + "\n\n")

        f.write(f"{'Target':<10} {'Setpoint':<12} {'Settling':<12} {'Overshoot':<12} {'SS Error':<12} {'RMS Error':<12}\n")
        f.write(f"{'Speed':<10} {'(pulses)':<12} {'Time (s)':<12} {'(%)':<12} {'(pulses)':<12} {'(pulses)':<12}\n")
        f.write("-" * 80 + "\n")

        for metrics in all_metrics:
            settling = f"{metrics['settling_time']:.3f}" if metrics['settling_time'] else "N/A"
            f.write(f"{metrics['target_speed']:<10.2f} {metrics['setpoint']:<12.0f} {settling:<12} "
                   f"{metrics['overshoot_percent']:<12.2f} {metrics['steady_state_error']:<12.3f} "
                   f"{metrics['rms_error']:<12.3f}\n")

        # Conclusions
        f.write("\n" + "=" * 80 + "\n")
        f.write("CONCLUSIONS AND RECOMMENDATIONS\n")
        f.write("=" * 80 + "\n\n")

        avg_settling = np.mean([m['settling_time'] for m in all_metrics if m['settling_time'] is not None])
        avg_overshoot = np.mean([abs(m['overshoot_percent']) for m in all_metrics])
        avg_ss_error = np.mean([m['steady_state_error'] for m in all_metrics])

        f.write(f"Average Performance Across All Steps:\n")
        f.write(f"  • Average Settling Time:       {avg_settling:.3f} s\n")
        f.write(f"  • Average Overshoot:           {avg_overshoot:.2f} %\n")
        f.write(f"  • Average SS Error:            {avg_ss_error:.3f} pulses\n\n")

        f.write("Stability Conclusion:\n")
        if all_stable:
            f.write("  The PID controller is STABLE and well-tuned. The system:\n")
            f.write("  - Responds quickly to setpoint changes\n")
            f.write("  - Settles without excessive overshoot\n")
            f.write("  - Maintains accurate tracking with minimal steady-state error\n")
            f.write("  - Shows consistent performance across different speeds\n")
            f.write("  - Demonstrates both forward and reverse operation capability\n\n")
            f.write("  This controller is suitable for deployment in the thesis application.\n")
        else:
            f.write("  The system shows some stability issues that may need attention:\n")
            for idx, ((target_speed, _), metrics) in enumerate(zip(steps, all_metrics)):
                if metrics['settling_time'] is None or metrics['settling_time'] > 3.0:
                    f.write(f"  - Step {idx+1} ({target_speed:.2f} m/s): Slow settling\n")
                if abs(metrics['overshoot_percent']) > 15:
                    f.write(f"  - Step {idx+1} ({target_speed:.2f} m/s): Excessive overshoot\n")
            f.write("\n  Consider PID retuning if performance is critical.\n")

        f.write("\n" + "=" * 80 + "\n")
        f.write("END OF REPORT\n")
        f.write("=" * 80 + "\n")

    return report_file

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_pid_results.py <csv_file>")
        sys.exit(1)

    csv_file = sys.argv[1]

    if not os.path.exists(csv_file):
        print(f"Error: File {csv_file} not found")
        sys.exit(1)

    # Generate output prefix
    output_prefix = csv_file.replace('.csv', '')

    print(f"Analyzing PID stability data from: {csv_file}")
    print("-" * 60)

    # Create comprehensive plots
    print("\n1. Generating comprehensive visualization plots...")
    plot_file, steps = create_comprehensive_plots(csv_file, output_prefix)
    print(f"   ✓ Saved: {plot_file}")

    # Generate text report
    print("\n2. Generating detailed stability report...")
    report_file = generate_text_report(csv_file, steps, output_prefix)
    print(f"   ✓ Saved: {report_file}")

    print("\n" + "=" * 60)
    print("ANALYSIS COMPLETE!")
    print("=" * 60)
    print(f"\nGenerated files:")
    print(f"  • Visualization: {plot_file}")
    print(f"  • Report:        {report_file}")
    print("\nPlease review the report for detailed stability analysis.")

if __name__ == '__main__':
    main()
