#!/usr/bin/env python3
"""Interactive ROI selection using matplotlib - supports zoom, pan, and rectangle drawing."""

import sys
sys.path.insert(0, '/workspaces/ros2_ws/src/mapir_camera')

from pathlib import Path
import json
import numpy as np
from mapir_camera_core.raw_survey3 import load_survey3_raw, demosaic_survey3_raw

import matplotlib.pyplot as plt
import matplotlib.patches as patches


class ROISelector:
    def __init__(self, image, num_panels=4):
        self.image = image
        self.num_panels = num_panels
        self.rois = []
        self.current_roi_start = None
        self.current_roi_rect = None

        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        self.im = self.ax.imshow(image)

        self.ax.set_title(
            f'Select {num_panels} Panel ROIs\n'
            'Scroll=Zoom | Pan (hold space) | Drag to draw ROI | Press Enter to finish',
            fontsize=12
        )

        # Connect events
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        self.pan_mode = False
        self.last_pos = None

        # Add colorbar for feedback
        plt.colorbar(self.im, ax=self.ax, label='Pixel Value')

    def on_scroll(self, event):
        """Handle zoom with scroll wheel."""
        if event.inaxes != self.ax:
            return

        # Get current axis limits
        cur_xlim = self.ax.get_xlim()
        cur_ylim = self.ax.get_ylim()

        xdata = event.xdata
        ydata = event.ydata

        # Zoom factor
        if event.button == 'up':
            scale_factor = 0.7  # Zoom in
        elif event.button == 'down':
            scale_factor = 1.4  # Zoom out
        else:
            return

        # Calculate new limits
        new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
        new_height = (cur_ylim[1] - cur_ylim[0]) * scale_factor

        relx = (cur_xlim[1] - xdata) / (cur_xlim[1] - cur_xlim[0])
        rely = (cur_ylim[1] - ydata) / (cur_ylim[1] - cur_ylim[0])

        self.ax.set_xlim([xdata - new_width * (1 - relx), xdata + new_width * relx])
        self.ax.set_ylim([ydata - new_height * (1 - rely), ydata + new_height * rely])

        self.fig.canvas.draw_idle()

    def on_press(self, event):
        """Handle mouse press."""
        if event.inaxes != self.ax:
            return

        if event.button == 3:  # Right click for pan
            self.pan_mode = True
            self.last_pos = (event.xdata, event.ydata)
        elif event.button == 1:  # Left click for ROI
            self.current_roi_start = (event.xdata, event.ydata)

    def on_release(self, event):
        """Handle mouse release."""
        if event.button == 3:  # Right click release
            self.pan_mode = False
            self.last_pos = None
        elif event.button == 1 and self.current_roi_start and event.inaxes == self.ax:
            # Finish ROI
            x1, y1 = self.current_roi_start
            x2, y2 = event.xdata, event.ydata

            x_min = int(min(x1, x2))
            x_max = int(max(x1, x2))
            y_min = int(min(y1, y2))
            y_max = int(max(y1, y2))

            if x_max > x_min and y_max > y_min:
                roi = [x_min, y_min, x_max, y_max]
                self.rois.append(roi)
                print(f"✓ ROI {len(self.rois)}/{self.num_panels}: {roi}")

                # Draw on plot
                rect = patches.Rectangle(
                    (x_min, y_min), x_max - x_min, y_max - y_min,
                    linewidth=2, edgecolor='cyan', facecolor='none'
                )
                self.ax.add_patch(rect)
                self.fig.canvas.draw_idle()

                if len(self.rois) >= self.num_panels:
                    print(f"\n✓ All {self.num_panels} ROIs selected!")
                    plt.close()

            self.current_roi_start = None
            if self.current_roi_rect:
                self.current_roi_rect.remove()
                self.current_roi_rect = None

    def on_motion(self, event):
        """Handle mouse motion."""
        if event.inaxes != self.ax:
            return

        # Pan mode
        if self.pan_mode and self.last_pos and event.button == 3:
            dx = event.xdata - self.last_pos[0]
            dy = event.ydata - self.last_pos[1]

            cur_xlim = self.ax.get_xlim()
            cur_ylim = self.ax.get_ylim()

            self.ax.set_xlim([cur_xlim[0] - dx, cur_xlim[1] - dx])
            self.ax.set_ylim([cur_ylim[0] - dy, cur_ylim[1] - dy])

            self.fig.canvas.draw_idle()
            self.last_pos = (event.xdata, event.ydata)

        # ROI preview
        elif self.current_roi_start and event.button == 1:
            x1, y1 = self.current_roi_start
            x2, y2 = event.xdata, event.ydata

            if self.current_roi_rect:
                self.current_roi_rect.remove()

            self.current_roi_rect = patches.Rectangle(
                (min(x1, x2), min(y1, y2)),
                abs(x2 - x1), abs(y2 - y1),
                linewidth=2, edgecolor='yellow', facecolor='none', linestyle='--'
            )
            self.ax.add_patch(self.current_roi_rect)
            self.fig.canvas.draw_idle()

    def on_key(self, event):
        """Handle keyboard events."""
        if event.key == 'enter' or event.key == 'escape':
            plt.close()

    def run(self):
        """Run the selector."""
        plt.tight_layout()
        plt.show()
        return self.rois


def select_rois_matplotlib(image_path, num_panels=4):
    """Select ROIs using matplotlib interactive viewer."""

    print(f"Loading image: {image_path}")

    # Load RAW image
    bayer = load_survey3_raw(Path(image_path))
    rgb = demosaic_survey3_raw(bayer, apply_white_balance=False, rgb_sensor=False)

    # Convert to 8-bit for display
    rgb_8bit = (rgb / 256).astype(np.uint8)

    print(f"Image shape: {rgb_8bit.shape}")
    print(f"\nInstructions:")
    print("  1. Use SCROLL WHEEL to ZOOM in/out")
    print("  2. Right-click and drag to PAN")
    print("  3. Left-click and drag to draw ROI rectangles")
    print(f"  4. Draw exactly {num_panels} rectangles (one per panel)")
    print("  5. Press Enter or close window when done\n")

    selector = ROISelector(rgb_8bit, num_panels=num_panels)
    rois = selector.run()

    return rois


if __name__ == '__main__':
    raw_image = '/outputs/reflectance_calibration/2026_0417_153120_001.RAW'

    rois = select_rois_matplotlib(raw_image, num_panels=4)

    if rois and len(rois) > 0:
        print(f"\n✓ Selected {len(rois)} ROIs")

        # Save to file
        output = {
            'rois': rois,
            'format': '[x_min, y_min, x_max, y_max]',
            'num_panels': len(rois)
        }

        output_file = Path('/outputs/reflectance_calibration/matplotlib_rois.json')
        output_file.parent.mkdir(parents=True, exist_ok=True)

        with open(output_file, 'w') as f:
            json.dump(output, f, indent=2)

        print(f"✓ Saved to {output_file}")
        print("\nROI coordinates:")
        for i, roi in enumerate(rois, 1):
            print(f"  Panel {i}: {roi}")
    else:
        print("No ROIs selected")
