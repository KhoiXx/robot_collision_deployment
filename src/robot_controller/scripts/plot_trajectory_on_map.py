#!/usr/bin/env python3
"""
Plot Robot Trajectory on Map
Reads trajectory CSV and map files, creates visualization for reports

Author: Claude
Date: 2025-11-23

Usage:
    python3 plot_trajectory_on_map.py --csv ~/trajectory_logs/trajectory_xxx.csv
    python3 plot_trajectory_on_map.py --csv trajectory.csv --map map_final.yaml --output report.png
"""
import argparse
import csv
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection
from PIL import Image
import yaml


class TrajectoryPlotter:
    def __init__(self, map_yaml_path=None):
        """Initialize with map configuration"""
        self.map_image = None
        self.map_origin = [-15.0, -15.0]
        self.map_resolution = 0.05
        self.map_height = 0
        self.map_width = 0

        if map_yaml_path:
            self.load_map(map_yaml_path)

    def load_map(self, yaml_path):
        """Load map from YAML and PGM files"""
        yaml_path = Path(yaml_path)

        # Read YAML
        with open(yaml_path, 'r') as f:
            map_config = yaml.safe_load(f)

        self.map_resolution = map_config['resolution']
        self.map_origin = map_config['origin'][:2]  # [x, y]

        # Load PGM image
        pgm_path = map_config['image']
        if not os.path.isabs(pgm_path):
            pgm_path = yaml_path.parent / pgm_path

        self.map_image = np.array(Image.open(pgm_path))
        self.map_height, self.map_width = self.map_image.shape[:2]

        print(f"Map loaded: {pgm_path}")
        print(f"  - Size: {self.map_width} x {self.map_height} pixels")
        print(f"  - Resolution: {self.map_resolution} m/pixel")
        print(f"  - Origin: {self.map_origin}")
        print(f"  - World size: {self.map_width * self.map_resolution:.1f} x {self.map_height * self.map_resolution:.1f} m")

    def world_to_pixel(self, x, y):
        """Convert world coordinates (m) to pixel coordinates"""
        pixel_x = (x - self.map_origin[0]) / self.map_resolution
        pixel_y = self.map_height - (y - self.map_origin[1]) / self.map_resolution
        return pixel_x, pixel_y

    def pixel_to_world(self, px, py):
        """Convert pixel coordinates to world coordinates (m)"""
        world_x = px * self.map_resolution + self.map_origin[0]
        world_y = (self.map_height - py) * self.map_resolution + self.map_origin[1]
        return world_x, world_y

    def load_trajectory(self, csv_path):
        """Load trajectory data from CSV"""
        trajectory = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                trajectory.append({
                    'time': float(row['time']),
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'theta': float(row['theta']),
                    'v': float(row['v']),
                    'w': float(row['w']),
                    'goal_x': float(row['goal_x']),
                    'goal_y': float(row['goal_y']),
                    'dist_to_goal': float(row['dist_to_goal'])
                })

        print(f"Trajectory loaded: {csv_path}")
        print(f"  - Points: {len(trajectory)}")
        if trajectory:
            print(f"  - Duration: {trajectory[-1]['time']:.1f} s")
            print(f"  - Start: ({trajectory[0]['x']:.2f}, {trajectory[0]['y']:.2f})")
            print(f"  - End: ({trajectory[-1]['x']:.2f}, {trajectory[-1]['y']:.2f})")
            print(f"  - Goal: ({trajectory[0]['goal_x']:.2f}, {trajectory[0]['goal_y']:.2f})")

        return trajectory

    def plot(self, trajectory, output_path=None, title=None, show_velocity=True,
             show_orientation=False, colorby='time', figsize=(12, 10)):
        """
        Plot trajectory on map

        Args:
            trajectory: List of trajectory points
            output_path: Path to save figure (None = show interactively)
            title: Figure title
            show_velocity: Show velocity color gradient
            show_orientation: Show orientation arrows
            colorby: 'time', 'velocity', or 'distance'
            figsize: Figure size (width, height) in inches
        """
        if not trajectory:
            print("Error: Empty trajectory!")
            return

        fig, ax = plt.subplots(figsize=figsize)

        # Draw map as background
        if self.map_image is not None:
            # Create RGB image for better visualization
            # PGM: 0=black(occupied), 205=gray(unknown), 254=white(free)
            map_rgb = np.zeros((*self.map_image.shape, 3), dtype=np.uint8)

            # Free space (white/light gray)
            free_mask = self.map_image > 200
            map_rgb[free_mask] = [240, 240, 240]

            # Occupied (black)
            occupied_mask = self.map_image < 50
            map_rgb[occupied_mask] = [40, 40, 40]

            # Unknown (gray)
            unknown_mask = (self.map_image >= 50) & (self.map_image <= 200)
            map_rgb[unknown_mask] = [180, 180, 180]

            # Calculate extent in world coordinates
            extent = [
                self.map_origin[0],  # left
                self.map_origin[0] + self.map_width * self.map_resolution,  # right
                self.map_origin[1],  # bottom
                self.map_origin[1] + self.map_height * self.map_resolution  # top
            ]

            ax.imshow(map_rgb, extent=extent, origin='lower', alpha=0.8)

        # Extract trajectory data
        xs = [p['x'] for p in trajectory]
        ys = [p['y'] for p in trajectory]
        times = [p['time'] for p in trajectory]
        velocities = [p['v'] for p in trajectory]
        distances = [p['dist_to_goal'] for p in trajectory]

        # Choose color mapping
        if colorby == 'time':
            colors = times
            cmap = 'viridis'
            clabel = 'Time (s)'
        elif colorby == 'velocity':
            colors = velocities
            cmap = 'RdYlGn'  # Red=slow, Green=fast
            clabel = 'Velocity (m/s)'
        else:  # distance
            colors = distances
            cmap = 'plasma'
            clabel = 'Distance to Goal (m)'

        # Create line segments for colored trajectory
        points = np.array([xs, ys]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # Normalize colors
        norm = plt.Normalize(min(colors), max(colors))
        lc = LineCollection(segments, cmap=cmap, norm=norm, linewidth=2.5, alpha=0.9)
        lc.set_array(np.array(colors[:-1]))
        line = ax.add_collection(lc)

        # Add colorbar
        cbar = fig.colorbar(line, ax=ax, shrink=0.8, pad=0.02)
        cbar.set_label(clabel, fontsize=10)

        # Mark start point
        start_x, start_y = xs[0], ys[0]
        ax.plot(start_x, start_y, 'go', markersize=15, markeredgecolor='darkgreen',
                markeredgewidth=2, label='Start', zorder=10)

        # Mark end point
        end_x, end_y = xs[-1], ys[-1]
        ax.plot(end_x, end_y, 'bs', markersize=12, markeredgecolor='darkblue',
                markeredgewidth=2, label='End', zorder=10)

        # Mark goal
        goal_x, goal_y = trajectory[0]['goal_x'], trajectory[0]['goal_y']
        ax.plot(goal_x, goal_y, 'r*', markersize=20, markeredgecolor='darkred',
                markeredgewidth=1, label='Goal', zorder=10)

        # Draw goal circle (goal_size = 0.15m from robot_env.py)
        goal_circle = patches.Circle((goal_x, goal_y), 0.15, fill=False,
                                      edgecolor='red', linewidth=2, linestyle='--',
                                      label='Goal Region')
        ax.add_patch(goal_circle)

        # Show orientation arrows (optional)
        if show_orientation:
            arrow_interval = max(1, len(trajectory) // 20)  # ~20 arrows
            for i in range(0, len(trajectory), arrow_interval):
                p = trajectory[i]
                dx = 0.15 * np.cos(p['theta'])
                dy = 0.15 * np.sin(p['theta'])
                ax.arrow(p['x'], p['y'], dx, dy, head_width=0.05,
                         head_length=0.03, fc='blue', ec='blue', alpha=0.5)

        # Calculate statistics
        total_distance = sum(np.sqrt((xs[i+1]-xs[i])**2 + (ys[i+1]-ys[i])**2)
                            for i in range(len(xs)-1))
        avg_velocity = np.mean(velocities)
        max_velocity = np.max(velocities)
        final_dist = distances[-1]
        total_time = times[-1]

        # Add statistics text box
        stats_text = (
            f"Duration: {total_time:.1f} s\n"
            f"Path Length: {total_distance:.2f} m\n"
            f"Avg Velocity: {avg_velocity:.2f} m/s\n"
            f"Max Velocity: {max_velocity:.2f} m/s\n"
            f"Final Distance: {final_dist:.3f} m"
        )
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', bbox=props, family='monospace')

        # Set labels and title
        ax.set_xlabel('X (m)', fontsize=11)
        ax.set_ylabel('Y (m)', fontsize=11)

        if title:
            ax.set_title(title, fontsize=14, fontweight='bold')
        else:
            ax.set_title('Robot Navigation Trajectory', fontsize=14, fontweight='bold')

        # Legend
        ax.legend(loc='upper right', fontsize=9)

        # Equal aspect ratio
        ax.set_aspect('equal')

        # Adjust view to trajectory with padding
        margin = 1.0  # meters
        ax.set_xlim(min(xs + [goal_x]) - margin, max(xs + [goal_x]) + margin)
        ax.set_ylim(min(ys + [goal_y]) - margin, max(ys + [goal_y]) + margin)

        # Grid
        ax.grid(True, alpha=0.3, linestyle='--')

        plt.tight_layout()

        # Save or show
        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight',
                       facecolor='white', edgecolor='none')
            print(f"Figure saved: {output_path}")
        else:
            plt.show()

        plt.close()

    def plot_multiple(self, trajectories, labels, output_path=None, title=None):
        """Plot multiple trajectories for comparison"""
        if not trajectories:
            print("Error: No trajectories!")
            return

        fig, ax = plt.subplots(figsize=(12, 10))

        # Draw map
        if self.map_image is not None:
            map_rgb = np.zeros((*self.map_image.shape, 3), dtype=np.uint8)
            map_rgb[self.map_image > 200] = [240, 240, 240]
            map_rgb[self.map_image < 50] = [40, 40, 40]
            map_rgb[(self.map_image >= 50) & (self.map_image <= 200)] = [180, 180, 180]

            extent = [
                self.map_origin[0],
                self.map_origin[0] + self.map_width * self.map_resolution,
                self.map_origin[1],
                self.map_origin[1] + self.map_height * self.map_resolution
            ]
            ax.imshow(map_rgb, extent=extent, origin='lower', alpha=0.8)

        # Colors for different trajectories
        colors = plt.cm.tab10(np.linspace(0, 1, len(trajectories)))

        all_xs, all_ys = [], []

        for traj, label, color in zip(trajectories, labels, colors):
            xs = [p['x'] for p in traj]
            ys = [p['y'] for p in traj]
            all_xs.extend(xs)
            all_ys.extend(ys)

            ax.plot(xs, ys, '-', color=color, linewidth=2, label=label, alpha=0.8)
            ax.plot(xs[0], ys[0], 'o', color=color, markersize=10)
            ax.plot(xs[-1], ys[-1], 's', color=color, markersize=8)

        # Goal (assuming same goal for all)
        if trajectories[0]:
            goal_x, goal_y = trajectories[0][0]['goal_x'], trajectories[0][0]['goal_y']
            ax.plot(goal_x, goal_y, 'r*', markersize=20, label='Goal')
            goal_circle = patches.Circle((goal_x, goal_y), 0.15, fill=False,
                                          edgecolor='red', linewidth=2, linestyle='--')
            ax.add_patch(goal_circle)

        ax.set_xlabel('X (m)', fontsize=11)
        ax.set_ylabel('Y (m)', fontsize=11)
        ax.set_title(title or 'Trajectory Comparison', fontsize=14, fontweight='bold')
        ax.legend(loc='upper right')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3, linestyle='--')

        margin = 1.0
        if all_xs and all_ys:
            ax.set_xlim(min(all_xs) - margin, max(all_xs) + margin)
            ax.set_ylim(min(all_ys) - margin, max(all_ys) + margin)

        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"Figure saved: {output_path}")
        else:
            plt.show()

        plt.close()


def main():
    parser = argparse.ArgumentParser(description='Plot robot trajectory on map')
    parser.add_argument('--csv', '-c', required=True, help='Trajectory CSV file')
    parser.add_argument('--map', '-m', default=None,
                       help='Map YAML file (default: auto-detect from robot_controller)')
    parser.add_argument('--output', '-o', default=None,
                       help='Output image path (default: show interactively)')
    parser.add_argument('--title', '-t', default=None, help='Figure title')
    parser.add_argument('--colorby', choices=['time', 'velocity', 'distance'],
                       default='time', help='Color trajectory by')
    parser.add_argument('--orientation', action='store_true',
                       help='Show orientation arrows')

    args = parser.parse_args()

    # Auto-detect map path
    if args.map is None:
        # Try common locations
        possible_maps = [
            Path(__file__).parent.parent / 'maps' / 'map_final.yaml',
            Path('/home/khoint/thesis/deployment/src/robot_controller/maps/map_final.yaml'),
        ]
        for map_path in possible_maps:
            if map_path.exists():
                args.map = str(map_path)
                print(f"Auto-detected map: {args.map}")
                break

    # Create plotter
    plotter = TrajectoryPlotter(args.map)

    # Load and plot trajectory
    trajectory = plotter.load_trajectory(args.csv)

    # Auto-generate output filename if not specified
    if args.output is None and not sys.stdout.isatty():
        csv_name = Path(args.csv).stem
        args.output = f"{csv_name}_plot.png"

    plotter.plot(
        trajectory,
        output_path=args.output,
        title=args.title,
        show_orientation=args.orientation,
        colorby=args.colorby
    )


if __name__ == "__main__":
    main()
