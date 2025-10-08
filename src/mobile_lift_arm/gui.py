#!/usr/bin/env python3
"""Tkinter based GUI for controlling a simulated scissor lift.

This module provides a simple graphical user interface built with
``tkinter`` that allows the user to command a scissor lift to a
desired height, execute predefined sequences of heights and monitor
the current state.  It utilises the :mod:`~scissor_lift_control.controller`
module to communicate with the ROS2 control node.  When run outside
of a ROS2 environment the ROS2 functionality is stubbed so that
syntax errors can still be detected.

The original GUI in the provided repository suffered from broken
indentation which prevented it from running.  This reimplementation
fixes those issues, wraps the code in a proper package and exposes
a ``main()`` function for use with ``console_scripts``.
"""

from __future__ import annotations

import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from typing import Dict, List

from controller import (
    ScissorLiftSequenceController,
    ScissorLiftController,
)

try:
    import rclpy  # type: ignore
    from std_msgs.msg import Float64  # type: ignore
except ImportError:
    # Provide minimal stubs when ROS2 is not available to allow
    # importing this module without raising ImportError.
    rclpy = None  # type: ignore
    Float64 = object  # type: ignore


class ScissorLiftGUI:
    """Graphical interface for interacting with a scissor lift controller."""

    def __init__(self) -> None:
        # Initialise ROS2 if available and create a node
        if rclpy is not None:
            rclpy.init()
            self.controller: ScissorLiftController = ScissorLiftSequenceController()
        else:
            # Create a dummy controller so attribute access works
            self.controller = ScissorLiftController()  # type: ignore

        # Current state variables (mirrored from the controller)
        self.current_position: float = 0.0
        self.current_height: float = 0.0

        # Build the GUI widgets
        self.create_gui()

        # Start a background thread to update from ROS2
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

    # ------------------------------------------------------------------
    # GUI construction
    # ------------------------------------------------------------------
    def create_gui(self) -> None:
        """Construct the main window and all child widgets."""
        self.root = tk.Tk()
        self.root.title('Scissor Lift Control Panel')
        self.root.geometry('600x500')
        self.root.resizable(True, True)

        # Main container frame
        main_frame = ttk.Frame(self.root, padding='10')
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Status display
        status_frame = ttk.LabelFrame(main_frame, text='Status', padding='10')
        status_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        ttk.Label(status_frame, text='Current Height:').grid(row=0, column=0, sticky=tk.W)
        self.height_label = ttk.Label(status_frame, text='0.00 m', font=('Arial', 12, 'bold'))
        self.height_label.grid(row=0, column=1, sticky=tk.W, padx=(10, 0))
        ttk.Label(status_frame, text='Hydraulic Position:').grid(row=1, column=0, sticky=tk.W)
        self.position_label = ttk.Label(status_frame, text='0.000 m', font=('Arial', 12, 'bold'))
        self.position_label.grid(row=1, column=1, sticky=tk.W, padx=(10, 0))

        # Manual control frame
        control_frame = ttk.LabelFrame(main_frame, text='Manual Control', padding='10')
        control_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        ttk.Label(control_frame, text='Target Height (m):').grid(row=0, column=0, sticky=tk.W)
        self.height_var = tk.DoubleVar(value=0.5)
        self.height_scale = ttk.Scale(control_frame, from_=0.3, to=2.5, orient=tk.HORIZONTAL,
                                      variable=self.height_var, length=300)
        self.height_scale.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=(10, 0))
        self.height_value_label = ttk.Label(control_frame, text='0.50 m')
        self.height_value_label.grid(row=0, column=2, padx=(10, 0))
        # Update displayed value when the slider moves
        self.height_var.trace_add('write', self.update_height_label)

        # Buttons for manual commands
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=1, column=0, columnspan=3, pady=(10, 0))
        ttk.Button(button_frame, text='Move to Target', command=self.move_to_target).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(button_frame, text='Emergency Stop', command=self.emergency_stop, style='Danger.TButton').pack(side=tk.LEFT, padx=(0, 10))

        # Quick preset buttons
        quick_frame = ttk.LabelFrame(main_frame, text='Quick Positions', padding='10')
        quick_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        positions: List[tuple[str, float]] = [
            ('Minimum', 0.3),
            ('Low', 0.8),
            ('Medium', 1.5),
            ('High', 2.0),
            ('Maximum', 2.5),
        ]
        for i, (name, height) in enumerate(positions):
            btn = ttk.Button(quick_frame, text=f'{name}\n({height}m)',
                             command=lambda h=height: self.quick_move(h))
            btn.grid(row=0, column=i, padx=5, pady=5)

        # Sequence buttons
        sequence_frame = ttk.LabelFrame(main_frame, text='Automated Sequences', padding='10')
        sequence_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        sequences = ['Demo', 'Test', 'Inspection']
        for i, seq in enumerate(sequences):
            btn = ttk.Button(sequence_frame, text=f'Run {seq}', command=lambda s=seq.lower(): self.run_sequence(s))
            btn.grid(row=0, column=i, padx=5, pady=5)

        # Activity log
        log_frame = ttk.LabelFrame(main_frame, text='Activity Log', padding='10')
        log_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 0))
        self.log_text = tk.Text(log_frame, height=8, width=70)
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))

        # Configure resizing behaviour
        main_frame.grid_rowconfigure(4, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)
        log_frame.grid_rowconfigure(0, weight=1)
        log_frame.grid_columnconfigure(0, weight=1)

        # Log startup message
        self.log('Scissor Lift Control Panel initialized')

    # ------------------------------------------------------------------
    # ROS2 communication and helper methods
    # ------------------------------------------------------------------
    def update_from_controller(self) -> None:
        """Synchronise displayed state with the controller."""
        self.current_position = self.controller.current_position
        self.current_height = self.controller.current_height
        self.height_label.config(text=f'{self.current_height:.2f} m')
        self.position_label.config(text=f'{self.current_position:.3f} m')

    def update_height_label(self, *args: object) -> None:
        """Update the numeric label next to the height slider."""
        height = float(self.height_var.get())
        self.height_value_label.config(text=f'{height:.2f} m')

    def move_to_target(self) -> None:
        """Send a command to move the lift to the height specified by the slider."""
        target_height = float(self.height_var.get())
        self.controller.set_target_height(target_height)
        self.log(f'Moving to height: {target_height:.2f}m (position: {self.controller.target_position:.3f}m)')

    def quick_move(self, height: float) -> None:
        """Move immediately to one of the predefined heights."""
        self.height_var.set(height)
        self.move_to_target()

    def emergency_stop(self) -> None:
        """Hold the current position immediately."""
        self.controller.emergency_stop()
        self.log('EMERGENCY STOP – Holding current position')
        messagebox.showwarning('Emergency Stop', 'Emergency stop activated!\nHolding current position.')

    def run_sequence(self, sequence_name: str) -> None:
        """Start a predefined sequence in a background thread."""
        def run_seq() -> None:
            sequences: Dict[str, List[float]] = {
                'demo': [0.5, 1.0, 1.5, 2.0, 1.5, 1.0, 0.5],
                'test': [0.3, 0.8, 1.2, 0.8, 0.3],
                'inspection': [1.8, 1.8, 1.8, 0.3],
            }
            if sequence_name not in sequences:
                self.log(f'ERROR: Unknown sequence {sequence_name}')
                return
            sequence = sequences[sequence_name]
            self.log(f'Starting {sequence_name} sequence...')
            for i, h in enumerate(sequence):
                self.root.after(0, lambda height=h: self.quick_move(height))
                self.root.after(0, lambda step=i + 1, total=len(sequence), height=h:
                                self.log(f'Sequence step {step}/{total}: {height}m'))
                time.sleep(3)
            self.root.after(0, lambda: self.log(f'{sequence_name} sequence completed'))

        threading.Thread(target=run_seq, daemon=True).start()

    def log(self, message: str) -> None:
        """Append a message to the log area with a timestamp."""
        timestamp = time.strftime('%H:%M:%S')
        log_message = f'[{timestamp}] {message}\n'
        def update_log() -> None:
            self.log_text.insert(tk.END, log_message)
            self.log_text.see(tk.END)
        self.root.after(0, update_log)

    def spin_ros(self) -> None:
        """Background thread to spin the ROS2 node and update the GUI."""
        while True:
            if rclpy is not None:
                try:
                    rclpy.spin_once(self.controller, timeout_sec=0.1)  # type: ignore[call-arg]
                except Exception as exc:
                    print(f'ROS error: {exc}')
            # Update displayed values periodically
            self.root.after(0, self.update_from_controller)
            time.sleep(0.1)

    def run(self) -> None:
        """Enter the Tk main loop and handle clean shutdown."""
        try:
            self.root.protocol('WM_DELETE_WINDOW', self.on_closing)
            self.root.mainloop()
        except KeyboardInterrupt:
            pass

    def on_closing(self) -> None:
        """Handle the window close event by shutting down ROS2 properly."""
        self.log('Shutting down control panel...')
        if rclpy is not None:
            self.controller.destroy_node()  # type: ignore[call-arg]
            rclpy.shutdown()
        self.root.destroy()


def main() -> None:
    """Entry point for the GUI console script."""
    gui = ScissorLiftGUI()
    gui.run()


if __name__ == '__main__':
    main()