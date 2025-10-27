import tkinter as tk
from tkinter import ttk
import math
import numpy as np

class SwerveDriveIK:
    def __init__(self, L=1.0, W=0.8):
        """
        Initialize swerve drive system
        L: half-length of chassis
        W: half-width of chassis
        """
        self.L = L
        self.W = W
        
        # Wheel positions (FR, FL, RL, RR)
        self.wheel_positions = np.array([
            [ L, -W],  # Wheel 1 (FR)
            [ L,  W],  # Wheel 2 (FL)
            [-L,  W],  # Wheel 3 (RL)
            [-L, -W]   # Wheel 4 (RR)
        ])
        
        self.wheel_labels = ['FR', 'FL', 'RL', 'RR']
        
    def inverse_kinematics(self, vx, vy, omega):
        """
        Calculate inverse kinematics for swerve drive
        
        Args:
            vx: linear velocity in x-direction (forward)
            vy: linear velocity in y-direction (left)
            omega: angular velocity (rad/s)
        
        Returns:
            wheel_angles: steering angles for each wheel (rad)
            wheel_speeds: drive speeds for each wheel
            sign_multipliers: direction multipliers after normalization
        """
        wheel_vx = []
        wheel_vy = []
        wheel_angles_raw = []
        wheel_angles_normalized = []
        wheel_speeds = []
        sign_multipliers = []
        
        for i, (xi, yi) in enumerate(self.wheel_positions):
            # Step 1: Calculate wheel velocity components
            vxi = vx - omega * yi
            vyi = vy + omega * xi
            
            wheel_vx.append(vxi)
            wheel_vy.append(vyi)
            
            # Step 2: Calculate raw steering angle
            theta_raw = math.atan2(vyi, vxi)
            wheel_angles_raw.append(theta_raw)
            
            # Step 3: Normalize to steering range (±π/2)
            if -math.pi/2 <= theta_raw <= math.pi/2:
                theta_norm = theta_raw
                sign_mult = 1
            elif theta_raw > math.pi/2:
                theta_norm = theta_raw - math.pi
                sign_mult = -1
            else:  # theta_raw < -π/2
                theta_norm = theta_raw + math.pi
                sign_mult = -1
                
            wheel_angles_normalized.append(theta_norm)
            sign_multipliers.append(sign_mult)
            
            # Step 4: Calculate drive speed
            speed = sign_mult * math.sqrt(vxi**2 + vyi**2)
            wheel_speeds.append(speed)
        
        return (np.array(wheel_angles_normalized), np.array(wheel_speeds), 
                np.array(sign_multipliers), np.array(wheel_vx), np.array(wheel_vy))

class SwerveDriveGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("4-Wheel Swerve Drive Inverse Kinematics")
        self.root.geometry("1400x900")
        
        # Initialize swerve drive system
        self.swerve = SwerveDriveIK(L=1.0, W=0.8)
        
        # Canvas dimensions
        self.canvas_width = 500
        self.canvas_height = 400
        self.scale = 80  # pixels per meter
        self.center_x = self.canvas_width // 2
        self.center_y = self.canvas_height // 2
        
        # Current values
        self.vx = tk.DoubleVar(value=1.0)
        self.vy = tk.DoubleVar(value=0.5)
        self.omega = tk.DoubleVar(value=0.3)
        
        self.setup_gui()
        self.update_visualization()
        
    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left frame for visualization
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Canvas for drawing
        self.canvas = tk.Canvas(left_frame, width=self.canvas_width, height=self.canvas_height,
                               bg='white', relief=tk.RAISED, borderwidth=2)
        self.canvas.pack(pady=10)
        
        # Control frame
        control_frame = ttk.LabelFrame(left_frame, text="Controls")
        control_frame.pack(fill=tk.X, pady=10)
        
        # Sliders
        self.create_slider(control_frame, "Vx (m/s)", self.vx, -2.0, 2.0, 0, 0)
        self.create_slider(control_frame, "Vy (m/s)", self.vy, -2.0, 2.0, 1, 0)
        self.create_slider(control_frame, "ω (rad/s)", self.omega, -1.0, 1.0, 2, 0)
        
        # Preset buttons
        preset_frame = ttk.Frame(control_frame)
        preset_frame.grid(row=3, column=0, columnspan=2, pady=10)
        
        ttk.Button(preset_frame, text="Forward", command=self.set_forward).pack(side=tk.LEFT, padx=5)
        ttk.Button(preset_frame, text="Strafe", command=self.set_strafe).pack(side=tk.LEFT, padx=5)
        ttk.Button(preset_frame, text="Rotate", command=self.set_rotate).pack(side=tk.LEFT, padx=5)
        ttk.Button(preset_frame, text="Complex", command=self.set_complex).pack(side=tk.LEFT, padx=5)
        
        # Right frame for data
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=10)
        
        # Results table
        self.create_results_table(right_frame)
        
        # Algorithm description
        self.create_algorithm_description(right_frame)
        
    def create_slider(self, parent, label, variable, from_, to, row, col):
        ttk.Label(parent, text=label).grid(row=row, column=col, sticky=tk.W, padx=5)
        
        slider_frame = ttk.Frame(parent)
        slider_frame.grid(row=row, column=col+1, sticky=tk.EW, padx=5, pady=2)
        
        slider = ttk.Scale(slider_frame, from_=from_, to=to, variable=variable, 
                          orient=tk.HORIZONTAL, length=200, command=self.on_slider_change)
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        value_label = ttk.Label(slider_frame, text=f"{variable.get():.2f}", width=6)
        value_label.pack(side=tk.RIGHT)
        
        # Store reference to update label
        variable.label = value_label
        
    def create_results_table(self, parent):
        table_frame = ttk.LabelFrame(parent, text="Inverse Kinematics Results")
        table_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Create treeview for table
        columns = ('Wheel', 'Position', 'Angle (°)', 'Speed', 'Sign', 'Vx', 'Vy')
        self.tree = ttk.Treeview(table_frame, columns=columns, show='headings', height=6)
        
        # Define headings
        for col in columns:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=80, anchor=tk.CENTER)
        
        self.tree.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Add scrollbar
        scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.tree.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.configure(yscrollcommand=scrollbar.set)
        
    def create_algorithm_description(self, parent):
        algo_frame = ttk.LabelFrame(parent, text="Algorithm Steps")
        algo_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        algorithm_text = """Algorithm Steps:

1. Calculate wheel velocity components:
   vxᵢ = vx - ω × yᵢ
   vyᵢ = vy + ω × xᵢ

2. Calculate raw steering angles:
   θᵢ = atan2(vyᵢ, vxᵢ)

3. Normalize to steering range (±90°):
   • Keep angle if -π/2 ≤ θ ≤ π/2 (sign = +1)
   • Subtract π if θ > π/2 (sign = -1)
   • Add π if θ < -π/2 (sign = -1)

4. Calculate drive speeds:
   speedᵢ = signᵢ × √(vxᵢ² + vyᵢ²)

Wheel Positions:
   FR: (+L, -W)    FL: (+L, +W)
   RR: (-L, -W)    RL: (-L, +W)

Coordinate System:
   X-axis: Forward (red arrow)
   Y-axis: Left (green arrow)
   Origin: Chassis center"""
        
        text_widget = tk.Text(algo_frame, wrap=tk.WORD, height=20, width=40, font=('Courier', 9))
        text_widget.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        text_widget.insert(tk.END, algorithm_text)
        text_widget.config(state=tk.DISABLED)
        
    def on_slider_change(self, event=None):
        # Update value labels
        self.vx.label.config(text=f"{self.vx.get():.2f}")
        self.vy.label.config(text=f"{self.vy.get():.2f}")
        self.omega.label.config(text=f"{self.omega.get():.2f}")
        
        # Update visualization
        self.update_visualization()
        
    def set_forward(self):
        self.vx.set(1.5)
        self.vy.set(0.0)
        self.omega.set(0.0)
        self.on_slider_change()
        
    def set_strafe(self):
        self.vx.set(0.0)
        self.vy.set(1.0)
        self.omega.set(0.0)
        self.on_slider_change()
        
    def set_rotate(self):
        self.vx.set(0.0)
        self.vy.set(0.0)
        self.omega.set(0.5)
        self.on_slider_change()
        
    def set_complex(self):
        self.vx.set(1.0)
        self.vy.set(0.8)
        self.omega.set(0.3)
        self.on_slider_change()
        
    def update_visualization(self):
        # Clear canvas
        self.canvas.delete("all")
        
        # Get current values
        vx, vy, omega = self.vx.get(), self.vy.get(), self.omega.get()
        
        # Calculate inverse kinematics
        angles, speeds, signs, wheel_vx, wheel_vy = self.swerve.inverse_kinematics(vx, vy, omega)
        
        # Draw coordinate system
        self.draw_coordinate_system()
        
        # Draw chassis
        self.draw_chassis()
        
        # Draw velocity vector
        self.draw_velocity_vector(vx, vy)
        
        # Draw angular velocity indicator
        self.draw_angular_velocity(omega)
        
        # Draw wheels
        self.draw_wheels(angles, speeds, signs)
        
        # Update results table
        self.update_results_table(angles, speeds, signs, wheel_vx, wheel_vy)
        
        # Add title
        title = f"Swerve Drive: Vx={vx:.1f}, Vy={vy:.1f}, ω={omega:.2f}"
        self.canvas.create_text(self.canvas_width//2, 20, text=title, font=('Arial', 12, 'bold'))
        
    def draw_coordinate_system(self):
        # X-axis (forward, red)
        x_end = self.center_x + 40
        self.canvas.create_line(self.center_x, self.center_y, x_end, self.center_y, 
                               fill='red', width=2, arrow=tk.LAST)
        self.canvas.create_text(x_end + 25, self.center_y - 10, text="+X\nFRONT", 
                               fill='red', font=('Arial', 9, 'bold'))
        
        # Y-axis (left, green)
        y_end = self.center_y - 40
        self.canvas.create_line(self.center_x, self.center_y, self.center_x, y_end, 
                               fill='green', width=2, arrow=tk.LAST)
        self.canvas.create_text(self.center_x + 25, y_end, text="+Y", 
                               fill='green', font=('Arial', 9, 'bold'))
        
    def draw_chassis(self):
        # Convert chassis dimensions to canvas coordinates
        L_canvas = self.swerve.L * self.scale
        W_canvas = self.swerve.W * self.scale
        
        # Draw chassis rectangle
        x1 = self.center_x - L_canvas
        y1 = self.center_y - W_canvas
        x2 = self.center_x + L_canvas
        y2 = self.center_y + W_canvas
        
        self.canvas.create_rectangle(x1, y1, x2, y2, outline='black', width=2, fill='lightgray')
        
        # Add front indicator (arrow pointing to the front of the robot)
        front_center_x = self.center_x + L_canvas
        front_center_y = self.center_y
        
        # Draw a triangle pointing forward to indicate the front of the robot
        arrow_size = 15
        self.canvas.create_polygon(
            front_center_x, front_center_y,
            front_center_x - arrow_size, front_center_y - arrow_size//2,
            front_center_x - arrow_size, front_center_y + arrow_size//2,
            fill='red', outline='black', width=1
        )
        
        # Add "FRONT" label
        self.canvas.create_text(front_center_x, front_center_y - 20, 
                               text="FRONT", fill='red', font=('Arial', 10, 'bold'))
        
    def draw_velocity_vector(self, vx, vy):
        if abs(vx) > 0.01 or abs(vy) > 0.01:
            # Scale velocity for display
            scale = 30
            end_x = self.center_x + vx * scale
            end_y = self.center_y - vy * scale  # Negative because canvas Y is inverted
            
            self.canvas.create_line(self.center_x, self.center_y, end_x, end_y,
                                   fill='blue', width=3, arrow=tk.LAST)
            
            # Label
            label_x = end_x + 10 if vx > 0 else end_x - 10
            label_y = end_y - 10 if vy > 0 else end_y + 10
            self.canvas.create_text(label_x, label_y, text=f"V({vx:.1f}, {vy:.1f})",
                                   fill='blue', font=('Arial', 9, 'bold'))
            
    def draw_angular_velocity(self, omega):
        if abs(omega) > 0.01:
            # Draw arc to show rotation
            radius = 30
            x1 = self.center_x - radius
            y1 = self.center_y - radius
            x2 = self.center_x + radius
            y2 = self.center_y + radius
            
            # Draw arc (simplified as circle outline)
            self.canvas.create_oval(x1, y1, x2, y2, outline='purple', width=2)
            
            # Arrow indicator
            if omega > 0:  # Counter-clockwise
                arrow_x = self.center_x + radius * 0.7
                arrow_y = self.center_y - radius * 0.7
                text_pos = "↻"
            else:  # Clockwise
                arrow_x = self.center_x + radius * 0.7
                arrow_y = self.center_y + radius * 0.7
                text_pos = "↺"
            
            self.canvas.create_text(arrow_x, arrow_y, text=text_pos, 
                                   fill='purple', font=('Arial', 16, 'bold'))
            self.canvas.create_text(self.center_x + 50, self.center_y - 50, 
                                   text=f"ω={omega:.2f}", fill='purple', font=('Arial', 9, 'bold'))
            
    def draw_wheels(self, angles, speeds, signs):
        colors = ['red', 'blue', 'green', 'orange']
        
        for i, ((xi, yi), label, angle, speed, sign) in enumerate(zip(
            self.swerve.wheel_positions, self.swerve.wheel_labels, 
            angles, speeds, signs)):
            
            # Convert to canvas coordinates
            canvas_x = self.center_x + xi * self.scale
            canvas_y = self.center_y - yi * self.scale  # Negative because canvas Y is inverted
            
            # Draw wheel
            wheel_radius = 8
            self.canvas.create_oval(canvas_x - wheel_radius, canvas_y - wheel_radius,
                                   canvas_x + wheel_radius, canvas_y + wheel_radius,
                                   fill=colors[i], outline='black', width=2)
            
            # Draw wheel label and position
            label_y = canvas_y + 25
            self.canvas.create_text(canvas_x, label_y, 
                                   text=f"{label}\n({xi:+.1f}, {yi:+.1f})",
                                   font=('Arial', 8))
            
            # Draw wheel orientation vector if significant speed
            if abs(speed) > 0.01:
                vector_length = 20
                end_x = canvas_x + vector_length * math.cos(angle)
                end_y = canvas_y - vector_length * math.sin(angle)  # Negative for canvas
                
                self.canvas.create_line(canvas_x, canvas_y, end_x, end_y,
                                       fill=colors[i], width=3, arrow=tk.LAST)
                
                # Speed label
                speed_x = end_x + 15 * math.cos(angle)
                speed_y = end_y - 15 * math.sin(angle)
                self.canvas.create_text(speed_x, speed_y, text=f"{speed:.2f}",
                                       fill=colors[i], font=('Arial', 8, 'bold'))
                
    def update_results_table(self, angles, speeds, signs, wheel_vx, wheel_vy):
        # Clear existing items
        for item in self.tree.get_children():
            self.tree.delete(item)
            
        # Add new data
        for i in range(4):
            values = (
                self.swerve.wheel_labels[i],
                f"({self.swerve.wheel_positions[i][0]:+.1f}, {self.swerve.wheel_positions[i][1]:+.1f})",
                f"{math.degrees(angles[i]):+.1f}°",
                f"{speeds[i]:+.2f}",
                f"{signs[i]:+d}",
                f"{wheel_vx[i]:+.2f}",
                f"{wheel_vy[i]:+.2f}"
            )
            self.tree.insert('', tk.END, values=values)

def main():
    root = tk.Tk()
    app = SwerveDriveGUI(root)
    
    # Print some example calculations
    print("4-Wheel Swerve Drive Inverse Kinematics")
    print("=" * 50)
    
    swerve = SwerveDriveIK(L=1.0, W=0.8)
    test_cases = [
        (1.0, 0.0, 0.0, "Forward Motion"),
        (0.0, 1.0, 0.0, "Strafe Left"),
        (0.0, 0.0, 0.5, "Pure Rotation"),
        (1.0, 0.5, 0.3, "Complex Motion")
    ]
    
    for vx, vy, omega, description in test_cases:
        print(f"\n{description}: Vx={vx}, Vy={vy}, ω={omega}")
        angles, speeds, signs, _, _ = swerve.inverse_kinematics(vx, vy, omega)
        
        for i, (angle, speed, sign) in enumerate(zip(angles, speeds, signs)):
            wheel_name = swerve.wheel_labels[i]
            print(f"  {wheel_name}: θ={math.degrees(angle):+6.1f}°, speed={speed:+6.2f}, sign={sign:+2d}")
    
    print(f"\nGUI launched. Use sliders and preset buttons to experiment!")
    
    root.mainloop()

if __name__ == "__main__":
    main()