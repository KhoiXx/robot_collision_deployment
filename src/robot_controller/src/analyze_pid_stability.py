#!/usr/bin/env python3

import csv
import os
os.environ['MPLCONFIGDIR'] = '/tmp/matplotlib-{}'.format(os.getpid())
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec


class PIDStabilityAnalyzer:
    """Analyze PID stability from test data and generate comprehensive reports"""

    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.data = self.load_data()

    def load_data(self):
        """Load CSV data"""
        data = {
            'test_time': [],
            'timestamp_ms': [],
            'target_speed': [],
            'setpoint': [],
            'current': [],
            'error': [],
            'output': []
        }

        with open(self.csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                data['test_time'].append(float(row['test_time']))
                data['timestamp_ms'].append(float(row['timestamp_ms']))
                if 'target_speed' in row:
                    data['target_speed'].append(float(row['target_speed']))
                data['setpoint'].append(float(row['setpoint']))
                data['current'].append(float(row['current']))
                data['error'].append(float(row['error']))
                data['output'].append(float(row['output']))

        # Convert to numpy arrays
        for key in data:
            data[key] = np.array(data[key])

        return data

    def calculate_step_metrics(self, time, setpoint, actual):
        """
        Calculate step response metrics

        Returns:
            dict with:
                - rise_time: Time to go from 10% to 90% of setpoint
                - settling_time: Time to stay within 2% of setpoint
                - overshoot: Maximum overshoot percentage
                - steady_state_error: Final steady-state error
        """
        if len(setpoint) == 0 or len(actual) == 0:
            return None

        # Get final setpoint value
        final_setpoint = setpoint[-1]

        if abs(final_setpoint) < 1e-6:  # Avoid division by zero
            return None

        # Find indices where setpoint changes (detect step)
        step_changes = np.where(np.abs(np.diff(setpoint)) > 0.1)[0]

        metrics = []

        for i, step_idx in enumerate(step_changes):
            # Get segment after step change
            if i < len(step_changes) - 1:
                end_idx = step_changes[i + 1]
            else:
                end_idx = len(setpoint)

            seg_time = time[step_idx:end_idx] - time[step_idx]
            seg_setpoint = setpoint[step_idx:end_idx]
            seg_actual = actual[step_idx:end_idx]

            if len(seg_time) < 5:
                continue

            target = seg_setpoint[0]

            if abs(target) < 1e-6:
                continue

            # Rise time (10% to 90%)
            threshold_10 = 0.1 * target
            threshold_90 = 0.9 * target
            rise_start = np.where(seg_actual >= threshold_10)[0]
            rise_end = np.where(seg_actual >= threshold_90)[0]

            rise_time = None
            if len(rise_start) > 0 and len(rise_end) > 0:
                rise_time = seg_time[rise_end[0]] - seg_time[rise_start[0]]

            # Settling time (2% band)
            settling_band = 0.02 * abs(target)
            settled_indices = np.where(np.abs(seg_actual - target) <= settling_band)[0]

            settling_time = None
            if len(settled_indices) > 0:
                # Find first index where it stays settled
                for j in range(len(settled_indices) - 10):
                    if np.all(np.abs(seg_actual[settled_indices[j]:] - target) <= settling_band):
                        settling_time = seg_time[settled_indices[j]]
                        break

            # Overshoot
            if target > 0:
                overshoot_value = np.max(seg_actual) - target
            else:
                overshoot_value = target - np.min(seg_actual)

            overshoot_percent = (overshoot_value / abs(target)) * 100 if abs(target) > 1e-6 else 0

            # Steady-state error
            steady_state_error = abs(target - np.mean(seg_actual[-20:]))

            metrics.append({
                'target_speed': target,
                'rise_time': rise_time,
                'settling_time': settling_time,
                'overshoot_percent': overshoot_percent,
                'steady_state_error': steady_state_error
            })

        return metrics

    def plot_comprehensive_analysis(self, output_file=None):
        """Generate comprehensive analysis plots"""

        fig = plt.figure(figsize=(16, 12))
        gs = GridSpec(3, 2, figure=fig, hspace=0.3, wspace=0.3)

        time = self.data['test_time']
        setpoint = self.data['setpoint']
        current = self.data['current']
        error = self.data['error']
        output = self.data['output']

        # Plot 1: Setpoint vs Actual
        ax1 = fig.add_subplot(gs[0, :])
        ax1.plot(time, setpoint, 'r--', label='Setpoint', linewidth=2)
        ax1.plot(time, current, 'b-', label='Actual', linewidth=1.5, alpha=0.8)
        ax1.set_xlabel('Time (s)', fontsize=12)
        ax1.set_ylabel('Speed (PWM)', fontsize=12)
        ax1.set_title('PID Response: Setpoint vs Actual', fontsize=14, fontweight='bold')
        ax1.legend(fontsize=11)
        ax1.grid(True, alpha=0.3)

        # Plot 2: Error over time
        ax2 = fig.add_subplot(gs[1, 0])
        ax2.plot(time, error, 'g-', linewidth=1.5)
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('Error (PWM)', fontsize=12)
        ax2.set_title('Tracking Error', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)

        # Plot 3: Control Output
        ax3 = fig.add_subplot(gs[1, 1])
        ax3.plot(time, output, 'purple', linewidth=1.5)
        ax3.set_xlabel('Time (s)', fontsize=12)
        ax3.set_ylabel('Control Output (PWM)', fontsize=12)
        ax3.set_title('PID Control Output', fontsize=14, fontweight='bold')
        ax3.grid(True, alpha=0.3)

        # Plot 4: Error histogram
        ax4 = fig.add_subplot(gs[2, 0])
        ax4.hist(error, bins=50, color='orange', alpha=0.7, edgecolor='black')
        ax4.axvline(x=0, color='r', linestyle='--', linewidth=2)
        ax4.set_xlabel('Error (PWM)', fontsize=12)
        ax4.set_ylabel('Frequency', fontsize=12)
        ax4.set_title('Error Distribution', fontsize=14, fontweight='bold')
        ax4.grid(True, alpha=0.3, axis='y')

        # Plot 5: Statistics text
        ax5 = fig.add_subplot(gs[2, 1])
        ax5.axis('off')

        # Calculate statistics
        mean_error = np.mean(np.abs(error))
        std_error = np.std(error)
        max_error = np.max(np.abs(error))
        rmse = np.sqrt(np.mean(error**2))

        stats_text = f"""
        STABILITY METRICS
        ==================

        Error Statistics:
        • Mean Absolute Error: {mean_error:.3f}
        • Std Dev of Error: {std_error:.3f}
        • Max Absolute Error: {max_error:.3f}
        • RMSE: {rmse:.3f}

        Control Output:
        • Mean Output: {np.mean(output):.3f}
        • Output Range: [{np.min(output):.1f}, {np.max(output):.1f}]

        Test Duration: {time[-1]:.2f} seconds
        Sample Count: {len(time)}
        """

        ax5.text(0.1, 0.5, stats_text, fontsize=11, family='monospace',
                 verticalalignment='center', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        if output_file:
            plt.savefig(output_file, dpi=300, bbox_inches='tight')
            print(f"Plot saved to {output_file}")
        else:
            plt.show()

        return fig

    def generate_report(self, output_file=None):
        """Generate text report with stability analysis"""

        report = []
        report.append("=" * 70)
        report.append("PID STABILITY ANALYSIS REPORT")
        report.append("=" * 70)
        report.append(f"Data file: {self.csv_file}")
        report.append(f"Test duration: {self.data['test_time'][-1]:.2f} seconds")
        report.append(f"Sample count: {len(self.data['test_time'])}\n")

        # Error analysis
        error = self.data['error']
        report.append("-" * 70)
        report.append("ERROR ANALYSIS")
        report.append("-" * 70)
        report.append(f"Mean Absolute Error: {np.mean(np.abs(error)):.4f}")
        report.append(f"Standard Deviation: {np.std(error):.4f}")
        report.append(f"Max Absolute Error: {np.max(np.abs(error)):.4f}")
        report.append(f"RMSE: {np.sqrt(np.mean(error**2)):.4f}\n")

        # Step response metrics
        if 'target_speed' in self.data and len(self.data['target_speed']) > 0:
            metrics_list = self.calculate_step_metrics(
                self.data['test_time'],
                self.data['setpoint'],
                self.data['current']
            )

            if metrics_list:
                report.append("-" * 70)
                report.append("STEP RESPONSE METRICS")
                report.append("-" * 70)

                for i, metrics in enumerate(metrics_list):
                    report.append(f"\nStep {i+1} (Target: {metrics['target_speed']:.3f}):")
                    report.append(f"  • Rise Time: {metrics['rise_time']:.3f} s" if metrics['rise_time'] else "  • Rise Time: N/A")
                    report.append(f"  • Settling Time: {metrics['settling_time']:.3f} s" if metrics['settling_time'] else "  • Settling Time: N/A")
                    report.append(f"  • Overshoot: {metrics['overshoot_percent']:.2f}%")
                    report.append(f"  • Steady-State Error: {metrics['steady_state_error']:.4f}")

        # Stability assessment
        report.append("\n" + "-" * 70)
        report.append("STABILITY ASSESSMENT")
        report.append("-" * 70)

        mean_abs_error = np.mean(np.abs(error))
        std_error = np.std(error)

        if mean_abs_error < 5 and std_error < 10:
            stability = "EXCELLENT - System is highly stable"
        elif mean_abs_error < 10 and std_error < 20:
            stability = "GOOD - System is stable with minor oscillations"
        elif mean_abs_error < 20 and std_error < 30:
            stability = "ACCEPTABLE - System is stable but could be improved"
        else:
            stability = "POOR - System shows instability or large errors"

        report.append(f"Overall Stability: {stability}\n")

        report.append("=" * 70)

        report_text = "\n".join(report)
        print(report_text)

        if output_file:
            with open(output_file, 'w') as f:
                f.write(report_text)
            print(f"\nReport saved to {output_file}")

        return report_text


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_pid_stability.py <csv_file>")
        print("Example: python3 analyze_pid_stability.py pid_test_results/step_response_left_20241115_123456.csv")
        sys.exit(1)

    csv_file = sys.argv[1]

    if not os.path.exists(csv_file):
        print(f"Error: File not found: {csv_file}")
        sys.exit(1)

    # Create analyzer
    analyzer = PIDStabilityAnalyzer(csv_file)

    # Generate output filenames
    base_name = os.path.splitext(csv_file)[0]
    plot_file = f"{base_name}_analysis.png"
    report_file = f"{base_name}_report.txt"

    # Generate report
    print("Generating stability report...\n")
    analyzer.generate_report(output_file=report_file)

    # Generate plots
    print("\nGenerating analysis plots...")
    analyzer.plot_comprehensive_analysis(output_file=plot_file)

    print("\nAnalysis complete!")
    print(f"  • Report: {report_file}")
    print(f"  • Plots: {plot_file}")
