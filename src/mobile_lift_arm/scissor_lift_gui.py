#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading
import time

class ScissorLiftGUI:
    def __init__(self):
        # Initialize ROS2
        rclpy.init()
        self.node = Node('scissor_lift_gui')

        # Publishers
        self.hydraulic_cmd_pub = self.node.create_publisher(
            Float64, '/hydraulic_position_controller/command', 10)

        # Subscribers
        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Current state
        self.current_position = 0.0
        self.current_height = 0.0

        # Create GUI
        self.create_gui()

        # Start ROS2 spinning in separate thread
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

    def create_gui(self):
        """Create the main GUI window"""
        self.root = tk.Tk()
        self.root.title('Scissor Lift Control Panel')
        self.root.geometry('600x500')
        self.root.resizable(True, True)

        # Main frame
        main_frame = ttk.Frame(self.root, padding='10')
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Status frame
        status_frame = ttk.LabelFrame(main_frame, text='Status', padding='10')
        status_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))

        ttk.Label(status_frame, text='Current Height:').grid(row=0, column=0, sticky=tk.W)
        self.height_label = ttk.Label(status_frame, text='0.00 m', font=('Arial', 12, 'bold'))
        self.height_label.grid(row=0, column=1, sticky=tk.W, padx=(10, 0))

        ttk.Label(status_frame, text='Hydraulic Position:').grid(row=1, column=0, sticky=tk.W)
        self.position_label = ttk.Label(status_frame, text='0.000 m', font=('Arial', 12, 'bold'))
        self.position_label.grid(row=1, column=1, sticky=tk.W, padx=(10, 0))

        # Control frame
        control_frame = ttk.LabelFrame(main_frame, text='Manual Control', padding='10')
        control_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))

        # Height slider
        ttk.Label(control_frame, text='Target Height (m):').grid(row=0, column=0, sticky=tk.W)
        self.height_var = tk.DoubleVar(value=0.5)
        self.height_scale = ttk.Scale(
            control_frame, from_=0.3, to=2.5, orient=tk.HORIZONTAL, 
            variable=self.height_var, length=300
        )
        self.height_scale.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=(10, 0))

        self.height_value_label = ttk.Label(control_frame, text='0.50 m')
        self.height_value_label.grid(row=0, column=2, padx=(10, 0))

        # Update height label when slider changes
        self.height_var.trace('w', self.update_height_label)

        # Control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=1, column=0, columnspan=3, pady=(10, 0))

        ttk.Button(button_frame, text='Move to Target', command=self.move_to_target).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(button_frame, text='Emergency Stop', command=self.emergency_stop, 
                  style='Danger.TButton').pack(side=tk.LEFT, padx=(0, 10))

        # Quick position buttons
        quick_frame = ttk.LabelFrame(main_frame, text='Quick Positions', padding='10')
        quick_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))

        positions = [
            ('Minimum', 0.3),
            ('Low', 0.8),
            ('Medium', 1.5),
            ('High', 2.0),
            ('Maximum', 2.5)
        ]

        for i, (name, height) in enumerate(positions):
            btn = ttk.Button(quick_frame, text=f'{name}\n({height}m)', 
                            command=lambda h=height: self.quick_move(h))
            btn.grid(row=0, column=i, padx=5, pady=5)

        # Sequence control
        sequence_frame = ttk.LabelFrame(main_frame, text='Automated Sequences', padding='10')
        sequence_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))

        sequences = ['Demo', 'Test', 'Inspection']
        for i, seq in enumerate(sequences):
            btn = ttk.Button(sequence_frame, text=f'Run {seq}', 
                            command=lambda s=seq.lower(): self.run_sequence(s))
            btn.grid(row=0, column=i, padx=5, pady=5)

        # Log frame
        log_frame = ttk.LabelFrame(main_frame, text='Activity Log', padding='10')
        log_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 0))

        self.log_text = tk.Text(log_frame, height=8, width=70)
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)

        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))

        # Configure grid weights
        main_frame.grid_rowconfigure(4, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)
        log_frame.grid_rowconfigure(0, weight=1)
        log_frame.grid_columnconfigure(0, weight=1)

        # Initial log message
        self.log('Scissor Lift Control Panel initialized')

    def joint_state_callback(self, msg):
        """Update current state from joint state messages"""
        try:
            hydraulic_idx = msg.name.index('hydraulic_actuator')
            self.current_position = msg.position[hydraulic_idx]
            self.current_height = self.calculate_height_from_position(self.current_position)

            # Update GUI (thread-safe)
            self.root.after(0, self.update_status_display)

        except (ValueError, IndexError):
            pass

    def calculate_height_from_position(self, hydraulic_pos):
        """Calculate platform height from hydraulic cylinder position"""
        import math
        base_height = 0.3
        scissor_length = 1.2
        angle = math.atan2(hydraulic_pos + 0.8, 1.0)
        height = base_height + scissor_length * math.sin(angle)
        return max(0.3, min(2.5, height))

    def update_status_display(self):
        """Update status labels"""
        self.height_label.config(text=f'{self.current_height:.2f} m')
        self.position_label.config(text=f'{self.current_position:.3f} m')

    def update_height_label(self, *args):
        """Update the height value label"""
        height = self.height_var.get()
        self.height_value_label.config(text=f'{height:.2f} m')

    def move_to_target(self):
        """Move to target height from slider"""
        target_height = self.height_var.get()
        target_position = self.calculate_position_from_height(target_height)

        msg = Float64()
        msg.data = target_position
        self.hydraulic_cmd_pub.publish(msg)

        self.log(f'Moving to height: {target_height:.2f}m (position: {target_position:.3f}m)')

    def quick_move(self, height):
        """Quick move to predefined height"""
        self.height_var.set(height)
        self.move_to_target()

    def emergency_stop(self):
        """Emergency stop - hold current position"""
        msg = Float64()
        msg.data = self.current_position
        self.hydraulic_cmd_pub.publish(msg)

        self.log('EMERGENCY STOP - Holding current position')
        messagebox.showwarning('Emergency Stop', 'Emergency stop activated!\nHolding current position.')

    def run_sequence(self, sequence_name):
        """Run automated sequence"""
        sequences = {
            'demo': [0.5, 1.0, 1.5, 2.0, 1.5, 1.0, 0.5],
            'test': [0.3, 0.8, 1.2, 0.8, 0.3],
            'inspection': [1.8, 1.8, 1.8, 0.3]
        }

        if sequence_name not in sequences:
            self.log(f'ERROR: Unknown sequence {sequence_name}')
            return

        self.log(f'Starting {sequence_name} sequence...')

        # Run sequence in separate thread to avoid blocking GUI
        def run_seq():
            sequence = sequences[sequence_name]
            for i, height in enumerate(sequence):
                self.root.after(0, lambda h=height: self.quick_move(h))
                self.root.after(0, lambda s=i+1, t=len(sequence), h=height: 
                               self.log(f'Sequence step {s}/{t}: {h}m'))
                time.sleep(3)  # Wait between steps

            self.root.after(0, lambda: self.log(f'{sequence_name} sequence completed'))

        seq_thread = threading.Thread(target=run_seq, daemon=True)
        seq_thread.start()

    def calculate_position_from_height(self, target_height):
        """Calculate required hydraulic position for target height"""
        import math
        base_height = 0.3
        scissor_length = 1.2
        target_height = max(0.3, min(2.5, target_height))
        required_angle = math.asin((target_height - base_height) / scissor_length)
        hydraulic_pos = 1.0 * math.tan(required_angle) - 0.8
        return max(0.0, min(0.6, hydraulic_pos))

    def log(self, message):
        """Add message to activity log"""
        timestamp = time.strftime('%H:%M:%S')
        log_message = f'[{timestamp}] {message}\n'

        def update_log():
            self.log_text.insert(tk.END, log_message)
            self.log_text.see(tk.END)

        self.root.after(0, update_log)

    def spin_ros(self):
        """ROS2 spinning thread"""
        while rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception as e:
                print(f'ROS error: {e}')

    def run(self):
        """Start the GUI"""
        try:
            self.root.protocol('WM_DELETE_WINDOW', self.on_closing)
            self.root.mainloop()
        except KeyboardInterrupt:
            pass

    def on_closing(self):
        """Clean shutdown"""
        self.log('Shutting down control panel...')
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

def main():
    gui = ScissorLiftGUI()
    gui.run()

if __name__ == '__main__':
    main()
