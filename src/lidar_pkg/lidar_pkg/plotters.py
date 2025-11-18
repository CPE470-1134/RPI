import matplotlib.pyplot as plt
import numpy as np
from abc import ABC, abstractmethod

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


