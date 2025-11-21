import matplotlib.pyplot as plt
import numpy as np
from abc import ABC, abstractmethod
import os
from pathlib import Path

class BasePlotter(ABC):
    """A base class for creating Lidar plots."""

    def __init__(self, title="Lidar Scan"):
        self.fig, self.ax = plt.subplots()
        self.scat = self.ax.scatter([], [], c=[], cmap='viridis', s=5)
        self.ax.set_title(title)
        self._init_plot()
        plt.ion()
        plt.show()

    @abstractmethod
    def _init_plot(self):
        """Initializes the plot-specific settings."""
        pass

    @abstractmethod
    def update(self, points):
        """Updates the plot with new points."""
        pass

    def clear(self):
        self.scat.set_offsets(np.c_[[], []])
        self.scat.set_array(np.array([]))
        self.ax.figure.canvas.draw()
        self.ax.figure.canvas.flush_events()

    def close(self):
        plt.ioff()
        plt.close(self.fig)

    def save(self, filename):
        self.fig.savefig(filename)

class PolarPlotter(BasePlotter):
    """A plotter for creating polar Lidar plots."""

    def _init_plot(self):


        self.ax.set_ylim(0, 12000)  # Assuming max distance is 120000 mm

        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)

        # Re-create the figure and axes with polar projection
        self.fig.clear()
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.scat = self.ax.scatter([], [], c=[], cmap='viridis', s=5)
        self.ax.set_ylim(0, 6000)
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        self.ax.set_title("Lidar Scan (Polar)")

    def update(self, points):
        if not points:
            return

        # Polar plot expects angles in radians
        angles_rad = np.radians([p.angle for p in points])
        distances = [p.distance for p in points]
        intensities = [p.intensity for p in points]

        # Offsets in polar plot are (angle, radius)
        self.scat.set_offsets(np.c_[angles_rad, distances])
        self.scat.set_array(np.array(intensities))

        # Update the plot
        self.ax.figure.canvas.draw()
        self.ax.figure.canvas.flush_events()
        plt.pause(0.01)

class CartesianPlotter(BasePlotter):
    """A plotter for creating Cartesian Lidar plots."""
    
    def __init__(self):
        super().__init__("Lidar Scan (Cartesian)")

    def _init_plot(self):
        
        # Set limits assuming max distance is 12000 mm
        view = 5000  # 1000 mm = 1 m
        self.ax.set_xlim(-view, view)
        self.ax.set_ylim(-view, view)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.autoscale()     # fits axes to data
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_title("Lidar Scan (Cartesian)")

    def update(self, points):
        if not points:
            return

        # Convert angles to radians
        angles_rad = np.radians([p.angle for p in points])
        distances = np.array([p.distance for p in points])
        intensities = np.array([p.intensity for p in points])

        # Convert polar to cartesian coordinates
        x = distances * np.cos(angles_rad)
        y = distances * np.sin(angles_rad)

        self.scat.set_offsets(np.c_[x, y])
        self.scat.set_array(intensities)
        self.ax.figure.canvas.draw()
        self.ax.figure.canvas.flush_events()
        plt.pause(0.01)

def visualize_opening(point_cloud, opening_result, filename="opening_detection.png"):
    """
    Clean plot of LD19 aggregated point cloud with annotated opening.
    """

    plt.style.use("bmh")

    # Extract data arrays
    xs = np.array([p['x'] for p in point_cloud.points])
    ys = np.array([p['y'] for p in point_cloud.points])
    intens = np.array([p['intensity'] for p in point_cloud.points])

    # ----------------------------------------------------------
    # Create figure
    # ----------------------------------------------------------
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect("equal")
    ax.set_title("LD19 Aggregated Scan + Opening Detection")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.grid(True, linestyle='--', alpha=0.7)

    # Scatter all points
    sc = ax.scatter(xs, ys, c=intens, s=12, cmap="viridis", edgecolor="none")
    plt.colorbar(sc, label="Intensity")

    # ----------------------------------------------------------
    # Handle opening annotation
    # ----------------------------------------------------------
    if opening_result:
        p1, p2, gap = opening_result

        # Extract coordinates
        x1, y1, d1 = p1['x'], p1['y'], p1['distance']
        x2, y2, d2 = p2['x'], p2['y'], p2['distance']

        # Draw the opening line
        ax.plot([x1, x2], [y1, y2], "r-", linewidth=3, label=f"Opening: {gap:.0f} mm")
        ax.legend(loc="upper right")

        # Opening endpoints in a list for reuse
        edge_points = [(x1, y1, d1, "p1"), (x2, y2, d2, "p2")]

        # Annotate both edges in one loop
        for x, y, dist, tag in edge_points:
            ax.scatter([x], [y], c="red", marker="x", s=70)
            ax.text(
                x, y,
                f"({x:.0f}, {y:.0f})\n Dist: {dist:.0f} mm",
                fontsize=5, color="darkred", weight="bold",
                ha="left" if tag == "p1" else "right",
                va="bottom" if tag == "p1" else "top",
                bbox=dict(facecolor="white", alpha=0.8)
            )

    # ----------------------------------------------------------
    # Save image
    # ----------------------------------------------------------
    fig.savefig(filename, dpi=300, bbox_inches="tight")
    print(f"\n✓ LiDAR annotated plot saved: {filename}\n")

    plt.show(block=False)