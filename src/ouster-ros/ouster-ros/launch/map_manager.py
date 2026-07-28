#!/usr/bin/env python3
"""
Map Manager GUI — Floating Control Panel for Persistent SLAM
=============================================================
A small always-on-top toolbar that provides two buttons:

  🗺️ New Map  — Wipes the saved map files and resets the SLAM so
                 the robot starts mapping from scratch at the current
                 location. Use this when deploying to a new site.

  💾 Save Now — Manually triggers an immediate map save to disk
                 (in addition to the automatic save every 2 minutes).

How "New Map" works under the hood:
  1. Deletes all /home/robotics/maps/slam_map_*.pcd files from disk.
  2. Publishes SlamCommand 10 (ENABLE_SLAM_MAP_UPDATE) so the
     in-memory map can be freely overwritten with new data.
  3. Publishes SlamCommand 13 (RESET_ODOM) so the robot's pose
     resets to (0, 0, 0) at its current physical position.
  4. The SLAM immediately begins building a fresh map.
  5. The auto-save node will persist the new map within 2 minutes.
  6. On next boot, the config file restores expansion mode (1)
     automatically, protecting the new map from being overwritten.

No ROS dependencies are imported — all ROS interactions use
subprocess calls for maximum reliability and zero import issues.
"""
import tkinter as tk
from tkinter import messagebox
import subprocess
import os
import glob
import threading

# ─── Configuration ───────────────────────────────────────────
MAP_PATH = '/home/robotics/maps/slam_map'
# ─────────────────────────────────────────────────────────────


class MapManagerGUI:
    """Minimal floating toolbar for SLAM map management."""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Map Manager')
        self.root.attributes('-topmost', True)
        self.root.resizable(False, False)

        # ── Dark theme styling ──
        BG = '#2b2b2b'
        FG = '#e0e0e0'
        self.root.configure(bg=BG)

        frame = tk.Frame(self.root, bg=BG, padx=12, pady=8)
        frame.pack()

        # Title
        tk.Label(
            frame, text='SLAM Map Manager',
            font=('Helvetica', 11, 'bold'), bg=BG, fg=FG
        ).pack(pady=(0, 6))

        # ── Button row ──
        btn_frame = tk.Frame(frame, bg=BG)
        btn_frame.pack()

        # New Map button (red — destructive action)
        self.new_map_btn = tk.Button(
            btn_frame,
            text='🗺️  New Map',
            command=self.new_map,
            bg='#c0392b', fg='white', activebackground='#e74c3c',
            font=('Helvetica', 10, 'bold'),
            width=14, height=2,
            relief='flat', cursor='hand2'
        )
        self.new_map_btn.pack(side=tk.LEFT, padx=4)

        # Save Now button (blue — safe action)
        self.save_btn = tk.Button(
            btn_frame,
            text='💾  Save Now',
            command=self.save_now,
            bg='#2980b9', fg='white', activebackground='#3498db',
            font=('Helvetica', 10, 'bold'),
            width=14, height=2,
            relief='flat', cursor='hand2'
        )
        self.save_btn.pack(side=tk.LEFT, padx=4)

        # ── Status bar ──
        self.status = tk.Label(
            frame, text=self._get_initial_status(),
            font=('Helvetica', 9), bg=BG, fg='#888888'
        )
        self.status.pack(pady=(6, 0))

    # ────────────────────────────────────────────────────────
    #  Actions
    # ────────────────────────────────────────────────────────

    def new_map(self):
        """Delete saved maps and reset the SLAM for a fresh start."""
        if not messagebox.askyesno(
            'Start New Map',
            'This will DELETE the current saved map and reset\n'
            'the SLAM to start mapping from scratch.\n\n'
            'Use this when deploying to a new location.\n\n'
            'Continue?',
            icon='warning'
        ):
            return

        self._set_status('Clearing map...', '#f39c12')
        self._disable_buttons()

        def do_reset():
            try:
                # 1. Delete saved map files from disk
                deleted = 0
                for f in glob.glob(f'{MAP_PATH}_*.pcd'):
                    os.remove(f)
                    deleted += 1

                # 2. Switch SLAM to full map update mode (command 10)
                #    This allows the in-memory map to be completely
                #    overwritten with new keypoints.
                subprocess.run(
                    ['ros2', 'topic', 'pub', '-1',
                     '/slam_command', 'lidar_slam/msg/SlamCommand',
                     '{command: 10}'],
                    capture_output=True, timeout=10
                )

                # 3. Reset SLAM pose to origin (command 13)
                #    The robot's current position becomes (0, 0, 0).
                subprocess.run(
                    ['ros2', 'topic', 'pub', '-1',
                     '/slam_command', 'lidar_slam/msg/SlamCommand',
                     '{command: 13}'],
                    capture_output=True, timeout=10
                )

                self.root.after(0, lambda: (
                    self._set_status(
                        f'✓ Map cleared ({deleted} files removed) — mapping from scratch',
                        '#27ae60'
                    ),
                    self._enable_buttons()
                ))

            except Exception as e:
                self.root.after(0, lambda: (
                    self._set_status(f'⚠ Reset failed: {e}', '#e74c3c'),
                    self._enable_buttons()
                ))

        threading.Thread(target=do_reset, daemon=True).start()

    def save_now(self):
        """Manually trigger an immediate map save."""
        self._set_status('Saving map...', '#f39c12')
        self._disable_buttons()

        def do_save():
            try:
                result = subprocess.run(
                    ['ros2', 'service', 'call',
                     '/lidar_slam/save_pc', 'lidar_slam/srv/SavePc',
                     '{output_prefix_path: ' + MAP_PATH +
                     ', format: 2, filtered: true, fixed: true}'],
                    capture_output=True, text=True, timeout=30
                )

                if result.returncode == 0:
                    self.root.after(0, lambda: (
                        self._set_status('✓ Map saved to /home/robotics/maps/', '#27ae60'),
                        self._enable_buttons()
                    ))
                else:
                    self.root.after(0, lambda: (
                        self._set_status('⚠ Save failed — SLAM may not be ready', '#e74c3c'),
                        self._enable_buttons()
                    ))

            except subprocess.TimeoutExpired:
                self.root.after(0, lambda: (
                    self._set_status('⚠ Save timed out', '#e74c3c'),
                    self._enable_buttons()
                ))
            except Exception as e:
                self.root.after(0, lambda: (
                    self._set_status(f'⚠ Save error: {e}', '#e74c3c'),
                    self._enable_buttons()
                ))

        threading.Thread(target=do_save, daemon=True).start()

    # ────────────────────────────────────────────────────────
    #  Helpers
    # ────────────────────────────────────────────────────────

    def _get_initial_status(self):
        """Check if a saved map exists and report status."""
        map_files = glob.glob(f'{MAP_PATH}_*.pcd')
        if map_files:
            return f'Saved map found ({len(map_files)} files)'
        else:
            return 'No saved map — will map from scratch'

    def _set_status(self, text, color='#888888'):
        self.status.config(text=text, fg=color)

    def _disable_buttons(self):
        self.new_map_btn.config(state=tk.DISABLED)
        self.save_btn.config(state=tk.DISABLED)

    def _enable_buttons(self):
        self.new_map_btn.config(state=tk.NORMAL)
        self.save_btn.config(state=tk.NORMAL)

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    app = MapManagerGUI()
    app.run()
