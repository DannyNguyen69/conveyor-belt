import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import cv2
from PIL import Image, ImageTk
import requests
import threading
import time
from ultralytics import YOLO
import numpy as np
import os
from datetime import datetime

W_products=['W-Cpolygon','W-Circle','W-S-Circle','W-Trapezoid','W-Square']
R_products=['R-Cpolygon','R-Circle','R-S-Circle','R-Trapezoid','R-Square']

class ESP32ConveyorControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Conveyor Belt Control with Auto YOLO Detection")
        
        # Optimize window size
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        window_width = min(1200, screen_width - 50)
        window_height = min(700, screen_height - 50)
        
        x = (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2
        
        self.root.geometry(f"{window_width}x{window_height}+{x}+{y}")
        self.root.minsize(1000, 600)
        
        # ESP32 configuration
        self.esp32_ip = ""
        self.esp32_connected = False
        
        # Camera and YOLO Configuration
        self.camera = None
        self.yolo_model = None
        self.camera_running = False
        
        # Detection area configuration - AUTO MODE
        self.detection_corners = []
        self.detection_area_set = False
        self.setting_area = False
        self.auto_area_enabled = True  # Enable auto area by default
        
        # Area adjustment parameters
        self.area_width_ratio = 0.6   # 60% of frame width
        self.area_height_ratio = 0.4  # 40% of frame height
        self.area_x_ratio = 0.5       # Center X
        self.area_y_ratio = 0.5       # Center Y
        
        # Conveyor belt control
        self.conveyor_running = False
        self.auto_detection_enabled = False
        self.manual_stop_requested = False
        
        # Detection parameters
        self.detection_threshold = 0.5
        self.labels_outside_area = True
        
        # Scan timing and control - FIXED
        self.scan_delay = 3.0
        self.scan_timer_start = None
        self.is_scanning = False
        self.products_stable_in_area = False
        self.motor_stopped = False
        self.last_motor_state = None
        
        # Photo capture settings
        self.photo_save_path = "D:/cap_bc"
        self.capture_enabled = True
        self.last_capture_time = 0
        self.capture_cooldown = 5.0
        
        # Current servo angle
        self.current_servo_angle = tk.StringVar(value="90")
        
        # Status variables
        self.connection_status = tk.StringVar(value="Disconnected")
        self.conveyor_status = tk.StringVar(value="Stopped")
        self.detection_status = tk.StringVar(value="No detections")
        
        # Canvas dimensions
        self.canvas_width = 480
        self.canvas_height = 360
        
        # Threading lock
        self.request_lock = threading.Lock()
        
        # Create photo directory
        self.create_photo_directory()
        
        self.setup_gui()
        self.load_yolo_model()
        
    def create_photo_directory(self):
        """Create directory for saving photos"""
        if not os.path.exists(self.photo_save_path):
            os.makedirs(self.photo_save_path)

# =============================================================================================================== #
#                                                  GUI SETUP                                                     #
# =============================================================================================================== #             
    def setup_gui(self):
        # Main container with scrollbar
        main_container = ttk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # Horizontal layout with 2 main columns
        left_panel = ttk.Frame(main_container)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 2))
        
        right_panel = ttk.Frame(main_container)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(2, 0))
        
        self.setup_left_panel(left_panel)
        self.setup_right_panel(right_panel)
        
    def setup_left_panel(self, parent):
        # Camera section - compact
        camera_frame = ttk.LabelFrame(parent, text="YOLO Camera Detection - AUTO AREA", padding=3)
        camera_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 2))
        
        # Camera display with smaller size
        self.camera_canvas = tk.Canvas(camera_frame, bg="black", 
                                     width=self.canvas_width, height=self.canvas_height,
                                     highlightthickness=1, highlightbackground="gray")
        self.camera_canvas.pack(pady=2)
        self.camera_canvas.bind("<Button-1>", self.on_canvas_click)
        
        # Camera controls - horizontal layout to save space
        camera_controls = ttk.Frame(camera_frame)
        camera_controls.pack(fill=tk.X, pady=2)
        
        # Row 1: Camera buttons
        cam_row1 = ttk.Frame(camera_controls)
        cam_row1.pack(fill=tk.X, pady=1)
        
        self.start_camera_btn = tk.Button(cam_row1, text="Start Camera", 
                                          command=self.start_camera,
                                          bg="#4CAF50", fg="white",
                                          font=("Arial", 8, "bold"),
                                          relief=tk.RAISED, borderwidth=1,
                                          padx=8, pady=2)
        self.start_camera_btn.pack(side=tk.LEFT, padx=2)
        
        self.stop_camera_btn = tk.Button(cam_row1, text="Stop Camera", 
                                         command=self.stop_camera, state=tk.DISABLED,
                                         bg="#f44336", fg="white",
                                         font=("Arial", 8, "bold"),
                                         relief=tk.RAISED, borderwidth=1,
                                         padx=8, pady=2)
        self.stop_camera_btn.pack(side=tk.LEFT, padx=2)
        
        # Row 2: Detection area controls - AUTO MODE
        cam_row2 = ttk.Frame(camera_controls)
        cam_row2.pack(fill=tk.X, pady=1)
        
        ttk.Label(cam_row2, text="Area:", font=("Arial", 8)).pack(side=tk.LEFT)
        
        # Auto area button (primary)
        self.auto_area_btn = tk.Button(cam_row2, text="Auto Area", 
                                      command=self.auto_set_detection_area,
                                      bg="#4CAF50", fg="white",
                                      font=("Arial", 8, "bold"),
                                      relief=tk.RAISED, borderwidth=1,
                                      padx=6, pady=2)
        self.auto_area_btn.pack(side=tk.LEFT, padx=2)
        
        # Smart area button (uses YOLO analysis)
        self.smart_area_btn = tk.Button(cam_row2, text="Smart Area", 
                                       command=self.smart_set_detection_area,
                                       bg="#2196F3", fg="white",
                                       font=("Arial", 8, "bold"),
                                       relief=tk.RAISED, borderwidth=1,
                                       padx=6, pady=2)
        self.smart_area_btn.pack(side=tk.LEFT, padx=2)
        
        # Manual area button (backup)
        self.manual_area_btn = tk.Button(cam_row2, text="Manual", 
                                        command=self.start_setting_area,
                                        bg="#FF9800", fg="white",
                                        font=("Arial", 8, "bold"),
                                        relief=tk.RAISED, borderwidth=1,
                                        padx=6, pady=2)
        self.manual_area_btn.pack(side=tk.LEFT, padx=2)
        
        self.clear_area_btn = tk.Button(cam_row2, text="Clear", 
                                       command=self.clear_detection_area,
                                       bg="#9E9E9E", fg="white",
                                       font=("Arial", 8, "bold"),
                                       relief=tk.RAISED, borderwidth=1,
                                       padx=6, pady=2)
        self.clear_area_btn.pack(side=tk.LEFT, padx=2)
        
        self.area_status = ttk.Label(cam_row2, text="Auto area ready", 
                                    foreground="blue", font=("Arial", 7))
        self.area_status.pack(side=tk.LEFT, padx=10)
        
        # Row 3: Conveyor controls - BIG BUTTONS
        conv_row = ttk.Frame(camera_controls)
        conv_row.pack(fill=tk.X, pady=3)
        
        self.start_conveyor_btn = tk.Button(conv_row, text="START", 
                                           command=self.start_conveyor,
                                           bg="#4CAF50", fg="white",
                                           font=("Arial", 11, "bold"),
                                           relief=tk.RAISED, borderwidth=2,
                                           padx=20, pady=6)
        self.start_conveyor_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_conveyor_btn = tk.Button(conv_row, text="STOP", 
                                          command=self.stop_conveyor,
                                          bg="#f44336", fg="white",
                                          font=("Arial", 11, "bold"),
                                          relief=tk.RAISED, borderwidth=2,
                                          padx=20, pady=6, state=tk.DISABLED)
        self.stop_conveyor_btn.pack(side=tk.LEFT, padx=5)
        
        # Status display compact
        status_row = ttk.Frame(camera_controls)
        status_row.pack(fill=tk.X, pady=2)
        
        ttk.Label(status_row, text="Status:", font=("Arial", 8, "bold")).pack(side=tk.LEFT)
        conveyor_status_display = ttk.Label(status_row, textvariable=self.conveyor_status,
                                           font=("Arial", 8, "bold"), foreground="blue")
        conveyor_status_display.pack(side=tk.LEFT, padx=5)
        
        # Detection info - reduced height
        info_frame = ttk.LabelFrame(parent, text="Detection Info", padding=2)
        info_frame.pack(fill=tk.BOTH, expand=True, pady=(2, 0))
        
        self.detection_info = tk.Text(info_frame, height=6, wrap=tk.WORD, font=("Arial", 7))
        info_scrollbar = ttk.Scrollbar(info_frame, orient=tk.VERTICAL, command=self.detection_info.yview)
        self.detection_info.config(yscrollcommand=info_scrollbar.set)
        self.detection_info.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        info_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
    def setup_right_panel(self, parent):
        # Right panel with scrollable frame
        canvas = tk.Canvas(parent, width=280)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Mouse wheel scrolling
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Now add all controls to scrollable_frame
        
        # Connection section - ultra compact
        conn_frame = ttk.LabelFrame(scrollable_frame, text="ESP32 Connection", padding=3)
        conn_frame.pack(fill=tk.X, pady=2)
        
        # IP entry row
        ip_row = ttk.Frame(conn_frame)
        ip_row.pack(fill=tk.X, pady=1)
        ttk.Label(ip_row, text="IP:", font=("Arial", 8)).pack(side=tk.LEFT)
        self.ip_entry = ttk.Entry(ip_row, width=15, font=("Arial", 8))
        self.ip_entry.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        self.ip_entry.insert(0, "192.168.1.100")
        
        # Connection buttons row
        conn_btns = ttk.Frame(conn_frame)
        conn_btns.pack(fill=tk.X, pady=1)
        
        self.connect_btn = tk.Button(conn_btns, text="Connect", 
                                     command=self.connect_esp32,
                                     bg="#2196F3", fg="white",
                                     font=("Arial", 8, "bold"),
                                     relief=tk.RAISED, borderwidth=1,
                                     padx=6, pady=2)
        self.connect_btn.pack(side=tk.LEFT, padx=1)
        
        self.disconnect_btn = tk.Button(conn_btns, text="Disconnect", 
                                        command=self.disconnect_esp32, state=tk.DISABLED,
                                        bg="#FF9800", fg="white",
                                        font=("Arial", 8, "bold"),
                                        relief=tk.RAISED, borderwidth=1,
                                        padx=6, pady=2)
        self.disconnect_btn.pack(side=tk.LEFT, padx=1)
        
        # Status row
        status_conn = ttk.Frame(conn_frame)
        status_conn.pack(fill=tk.X, pady=1)
        ttk.Label(status_conn, text="ESP32:", font=("Arial", 7)).pack(side=tk.LEFT)
        connection_display = ttk.Label(status_conn, textvariable=self.connection_status, 
                                      foreground="red", font=("Arial", 7, "bold"))
        connection_display.pack(side=tk.LEFT, padx=3)
        
        # AUTO AREA ADJUSTMENT - NEW SECTION
        self.setup_area_controls(scrollable_frame)
        
        # Scan Settings - collapsible
        scan_frame = ttk.LabelFrame(scrollable_frame, text="Scan Settings", padding=3)
        scan_frame.pack(fill=tk.X, pady=2)
        
        scan_row = ttk.Frame(scan_frame)
        scan_row.pack(fill=tk.X, pady=1)
        ttk.Label(scan_row, text="Delay:", font=("Arial", 8)).pack(side=tk.LEFT)
        self.scan_delay_var = tk.DoubleVar(value=3.0)
        scan_scale = ttk.Scale(scan_row, from_=1.0, to=10.0, orient=tk.HORIZONTAL,
                              variable=self.scan_delay_var, length=120)
        scan_scale.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Label(scan_row, textvariable=self.scan_delay_var, font=("Arial", 7)).pack(side=tk.LEFT)
        
        # Photo Capture - compact
        photo_frame = ttk.LabelFrame(scrollable_frame, text="Photo Capture", padding=3)
        photo_frame.pack(fill=tk.X, pady=2)
        
        self.capture_enabled_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(photo_frame, text="Auto capture", 
                       variable=self.capture_enabled_var, 
                       style="Small.TCheckbutton").pack(anchor=tk.W)
        
        photo_btns = ttk.Frame(photo_frame)
        photo_btns.pack(fill=tk.X, pady=1)
        
        ttk.Button(photo_btns, text="Folder", 
                  command=self.set_photo_folder,
                  style="Small.TButton").pack(side=tk.LEFT, padx=1)
        ttk.Button(photo_btns, text="Capture", 
                  command=self.manual_capture,
                  style="Small.TButton").pack(side=tk.LEFT, padx=1)
        
        self.photo_count_var = tk.StringVar(value="Photos: 0")
        ttk.Label(photo_frame, textvariable=self.photo_count_var, font=("Arial", 7)).pack()
        
        # Servo Control - compact
        servo_frame = ttk.LabelFrame(scrollable_frame, text="Servo Control", padding=3)
        servo_frame.pack(fill=tk.X, pady=2)
        
        servo_row1 = ttk.Frame(servo_frame)
        servo_row1.pack(fill=tk.X, pady=1)
        ttk.Label(servo_row1, text="Angle:", font=("Arial", 8)).pack(side=tk.LEFT)
        self.servo_scale = ttk.Scale(servo_row1, from_=0, to=180, orient=tk.HORIZONTAL,
                                    variable=self.current_servo_angle, length=100)
        self.servo_scale.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Label(servo_row1, textvariable=self.current_servo_angle, font=("Arial", 7)).pack(side=tk.LEFT)
        
        servo_row2 = ttk.Frame(servo_frame)
        servo_row2.pack(fill=tk.X, pady=1)
        
        ttk.Button(servo_row2, text="Set", 
                  command=self.send_servo_angle,
                  style="Small.TButton").pack(side=tk.LEFT, padx=1)
        ttk.Button(servo_row2, text="0°", 
                  command=lambda: self.quick_servo(0),
                  style="Small.TButton").pack(side=tk.LEFT, padx=1)
        ttk.Button(servo_row2, text="90°", 
                  command=lambda: self.quick_servo(90),
                  style="Small.TButton").pack(side=tk.LEFT, padx=1)
        ttk.Button(servo_row2, text="180°", 
                  command=lambda: self.quick_servo(180),
                  style="Small.TButton").pack(side=tk.LEFT, padx=1)
        
        # Detection Config - compact
        config_frame = ttk.LabelFrame(scrollable_frame, text="Detection Config", padding=3)
        config_frame.pack(fill=tk.X, pady=2)
        
        thresh_row = ttk.Frame(config_frame)
        thresh_row.pack(fill=tk.X, pady=1)
        ttk.Label(thresh_row, text="Threshold:", font=("Arial", 8)).pack(side=tk.LEFT)
        self.threshold_var = tk.DoubleVar(value=0.1)
        threshold_scale = ttk.Scale(thresh_row, from_=0.1, to=0.9, orient=tk.HORIZONTAL,
                                   variable=self.threshold_var, length=100)
        threshold_scale.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Label(thresh_row, textvariable=self.threshold_var, font=("Arial", 7)).pack(side=tk.LEFT)
        
        det_status_row = ttk.Frame(config_frame)
        det_status_row.pack(fill=tk.X, pady=1)
        ttk.Label(det_status_row, text="Status:", font=("Arial", 8)).pack(side=tk.LEFT)
        detection_display = ttk.Label(det_status_row, textvariable=self.detection_status,
                                     foreground="green", font=("Arial", 7, "bold"))
        detection_display.pack(side=tk.LEFT, padx=3)
        
        # Manual Motor Control - compact
        manual_frame = ttk.LabelFrame(scrollable_frame, text="Manual Test", padding=3)
        manual_frame.pack(fill=tk.X, pady=2)
        
        manual_btns = ttk.Frame(manual_frame)
        manual_btns.pack(fill=tk.X, pady=1)
        
        ttk.Button(manual_btns, text="Motor HIGH", 
                  command=lambda: self.send_motor_signal("HIGH"),
                  style="Small.TButton").pack(side=tk.LEFT, padx=1)
        ttk.Button(manual_btns, text="Motor LOW", 
                  command=lambda: self.send_motor_signal("LOW"),
                  style="Small.TButton").pack(side=tk.LEFT, padx=1)
        
        # System log - compact
        log_frame = ttk.LabelFrame(scrollable_frame, text="System Log", padding=3)
        log_frame.pack(fill=tk.X, pady=2)
        
        # Compact log with scrollbar
        log_container = ttk.Frame(log_frame)
        log_container.pack(fill=tk.X)
        
        self.log_text = tk.Text(log_container, height=8, wrap=tk.WORD, font=("Arial", 7))
        log_scrollbar = ttk.Scrollbar(log_container, orient=tk.VERTICAL, command=self.log_text.yview)
        
        self.log_text.config(yscrollcommand=log_scrollbar.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Configure small button style
        style = ttk.Style()
        style.configure("Small.TButton", padding=(2, 2))
        style.configure("Small.TCheckbutton", padding=(2, 2))
        
        # Update photo count initially
        self.update_photo_count()

    def setup_area_controls(self, parent):
        """Add auto area adjustment controls"""
        area_frame = ttk.LabelFrame(parent, text="AUTO Area Adjustment", padding=3)
        area_frame.pack(fill=tk.X, pady=2)
        
        # Area size controls
        size_row1 = ttk.Frame(area_frame)
        size_row1.pack(fill=tk.X, pady=1)
        
        ttk.Label(size_row1, text="Width:", font=("Arial", 8)).pack(side=tk.LEFT)
        self.area_width_var = tk.DoubleVar(value=0.4)
        width_scale = ttk.Scale(size_row1, from_=0.2, to=0.9, orient=tk.HORIZONTAL,
                               variable=self.area_width_var, length=120,
                               command=self.on_area_adjustment)
        width_scale.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Label(size_row1, textvariable=self.area_width_var, font=("Arial", 7)).pack(side=tk.LEFT)
        
        size_row2 = ttk.Frame(area_frame)
        size_row2.pack(fill=tk.X, pady=1)
        
        ttk.Label(size_row2, text="Height:", font=("Arial", 8)).pack(side=tk.LEFT)
        self.area_height_var = tk.DoubleVar(value=0.4)
        height_scale = ttk.Scale(size_row2, from_=0.2, to=0.8, orient=tk.HORIZONTAL,
                                variable=self.area_height_var, length=120,
                                command=self.on_area_adjustment)
        height_scale.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Label(size_row2, textvariable=self.area_height_var, font=("Arial", 7)).pack(side=tk.LEFT)
        
        # Position controls
        pos_row1 = ttk.Frame(area_frame)
        pos_row1.pack(fill=tk.X, pady=1)
        
        ttk.Label(pos_row1, text="X Pos:", font=("Arial", 8)).pack(side=tk.LEFT)
        self.area_x_var = tk.DoubleVar(value=0.5)
        x_scale = ttk.Scale(pos_row1, from_=0.1, to=0.9, orient=tk.HORIZONTAL,
                           variable=self.area_x_var, length=120,
                           command=self.on_area_adjustment)
        x_scale.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Label(pos_row1, textvariable=self.area_x_var, font=("Arial", 7)).pack(side=tk.LEFT)
        
        pos_row2 = ttk.Frame(area_frame)
        pos_row2.pack(fill=tk.X, pady=1)
        
        ttk.Label(pos_row2, text="Y Pos:", font=("Arial", 8)).pack(side=tk.LEFT)
        self.area_y_var = tk.DoubleVar(value=0.5)
        y_scale = ttk.Scale(pos_row2, from_=0.1, to=0.9, orient=tk.HORIZONTAL,
                           variable=self.area_y_var, length=120,
                           command=self.on_area_adjustment)
        y_scale.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Label(pos_row2, textvariable=self.area_y_var, font=("Arial", 7)).pack(side=tk.LEFT)

# =============================================================================================================== #
#                                              AUTO AREA METHODS                                                 #
# =============================================================================================================== #
    
    def on_area_adjustment(self, value=None):
        """Called when area adjustment sliders change"""
        if self.camera_running and self.auto_area_enabled:
            self.root.after(100, self.auto_set_detection_area)  # Small delay to avoid too frequent updates
    
    def auto_set_detection_area(self):
        """Automatically set detection area with adjustable parameters"""
        if not self.camera_running:
            self.log_message("Camera not running - cannot set auto area")
            return False
        
        # Get slider values
        width_ratio = self.area_width_var.get()
        height_ratio = self.area_height_var.get()
        x_ratio = self.area_x_var.get()
        y_ratio = self.area_y_var.get()
        
        # Calculate area dimensions
        area_width = int(self.canvas_width * width_ratio)
        area_height = int(self.canvas_height * height_ratio)
        
        # Calculate center position
        center_x = int(self.canvas_width * x_ratio)
        center_y = int(self.canvas_height * y_ratio)
        
        # Calculate corners
        left = center_x - area_width // 2
        right = center_x + area_width // 2
        top = center_y - area_height // 2
        bottom = center_y + area_height // 2
        
        # Clamp to canvas bounds
        left = max(0, left)
        right = min(self.canvas_width, right)
        top = max(0, top)
        bottom = min(self.canvas_height, bottom)
        
        # Set corners (clockwise from top-left)
        self.detection_corners = [
            (left, top),      # Top-left
            (right, top),     # Top-right  
            (right, bottom),  # Bottom-right
            (left, bottom)    # Bottom-left
        ]
        
        self.detection_area_set = True
        self.setting_area = False
        self.auto_area_enabled = True
        
        # Draw the area
        self.draw_detection_area()
        
        self.area_status.config(text="Auto area set ✓", foreground="green")
        self.log_message(f"Auto area set: {area_width}x{area_height} at ({center_x},{center_y})")
        return True
    
    def smart_set_detection_area(self):
        """Set detection area based on YOLO detection analysis"""
        if not self.camera_running or not self.yolo_model:
            messagebox.showwarning("Warning", "Camera and YOLO model must be running")
            return False
        
        self.log_message("Analyzing detections for smart area placement...")
        self.area_status.config(text="Analyzing...", foreground="orange")
        
        def analyze_detections():
            detection_points = []
            frames_analyzed = 0
            target_frames = 20
            
            # Collect detection points over several frames
            while frames_analyzed < target_frames and self.camera_running:
                try:
                    ret, frame = self.camera.read()
                    if not ret:
                        continue
                        
                    frame = cv2.resize(frame, (self.canvas_width, self.canvas_height))
                    results = self.yolo_model(frame, conf=0.3)
                    
                    for r in results:
                        boxes = r.boxes
                        if boxes is not None:
                            for box in boxes:
                                cls = int(box.cls[0])
                                class_name = self.yolo_model.names[cls]
                                
                                # Only consider target classes
                                if class_name in ["R-Square", "W-Square"]:
                                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                    center_x = (x1 + x2) / 2
                                    center_y = (y1 + y2) / 2
                                    detection_points.append((center_x, center_y))
                    
                    frames_analyzed += 1
                    self.root.after(0, lambda: self.area_status.config(
                        text=f"Analyzing... {frames_analyzed}/{target_frames}", 
                        foreground="orange"))
                    time.sleep(0.1)
                    
                except Exception as e:
                    self.log_message(f"Error during smart analysis: {e}")
                    break
            
            # Process results
            if len(detection_points) < 3:
                self.log_message("Not enough detections found, using center area")
                self.root.after(0, self.auto_set_detection_area)
                return
            
            # Find bounding box of all detections with margin
            min_x = max(0, min(p[0] for p in detection_points) - 60)
            max_x = min(self.canvas_width, max(p[0] for p in detection_points) + 60)
            min_y = max(0, min(p[1] for p in detection_points) - 40)
            max_y = min(self.canvas_height, max(p[1] for p in detection_points) + 40)
            
            self.detection_corners = [
                (int(min_x), int(min_y)),
                (int(max_x), int(min_y)), 
                (int(max_x), int(max_y)),
                (int(min_x), int(max_y))
            ]
            
            self.detection_area_set = True
            self.setting_area = False
            self.auto_area_enabled = False  # Disable auto adjustments for smart area
            
            self.root.after(0, self.draw_detection_area)
            self.root.after(0, lambda: self.area_status.config(
                text="Smart area set ✓", foreground="green"))
            self.log_message(f"Smart area set based on {len(detection_points)} detections")
        
        # Run analysis in background thread
        threading.Thread(target=analyze_detections, daemon=True).start()
        return True
    
    def draw_detection_area(self):
        """Draw detection area on canvas"""
        self.camera_canvas.delete("detection_area")
        self.camera_canvas.delete("detection_point")
        
        if len(self.detection_corners) < 4:
            return
            
        # Draw area outline
        for i in range(4):
            x1, y1 = self.detection_corners[i]
            x2, y2 = self.detection_corners[(i + 1) % 4]
            self.camera_canvas.create_line(x1, y1, x2, y2,
                                         fill="green", width=2,
                                         tags="detection_area")
        
        # Draw corner points
        for i, (x, y) in enumerate(self.detection_corners):
            self.camera_canvas.create_oval(x-4, y-4, x+4, y+4,
                                         fill="red", outline="white", width=1,
                                         tags="detection_point")
            self.camera_canvas.create_text(x+8, y-8, text=str(i+1),
                                         fill="yellow", font=("Arial", 10, "bold"),
                                         tags="detection_point")

# =============================================================================================================== #
#                                                  PROCESSING                                                    #
# =============================================================================================================== #    
    def load_yolo_model(self):
        try:
            self.yolo_model = YOLO(r'D:\3. PROJECT\2. ĐỒ ÁN BĂNG CHUYỀN\BANG_CHUYEN\train10-20250812T055215Z-1-001\train10\weights\best.pt')
            self.log_message("YOLO model loaded successfully!")
            self.update_detection_info("YOLO model ready. Auto area enabled.")
        except Exception as e:
            error_msg = f"Error loading YOLO model: {str(e)}"
            self.log_message(error_msg)
            self.update_detection_info(error_msg)
            
    def start_camera(self):
        try:
            self.camera = cv2.VideoCapture(0)
            if not self.camera.isOpened():
                messagebox.showerror("Error", "Cannot open camera")
                return
                
            self.camera_running = True
            self.start_camera_btn.config(state=tk.DISABLED)
            self.stop_camera_btn.config(state=tk.NORMAL)
            
            self.camera_thread = threading.Thread(target=self.update_camera, daemon=True)
            self.camera_thread.start()
            
            self.log_message("Camera started successfully")
            
            # AUTO-SET DETECTION AREA after camera stabilizes
            self.root.after(1500, self.auto_set_detection_area)  # Wait 1.5 seconds
            
        except Exception as e:
            messagebox.showerror("Error", f"Error starting camera: {str(e)}")
            self.log_message(f"Camera start error: {str(e)}")
            
    def stop_camera(self):
        self.camera_running = False
        if self.camera:
            self.camera.release()
        self.camera_canvas.delete("all")
        self.start_camera_btn.config(state=tk.NORMAL)
        self.stop_camera_btn.config(state=tk.DISABLED)
        self.detection_area_set = False
        self.area_status.config(text="Camera stopped", foreground="red")
        self.log_message("Camera stopped")
        
    def start_setting_area(self):
        """Manual area setting (backup method)"""
        if not self.camera_running:
            messagebox.showwarning("Warning", "Please start camera first")
            return
            
        self.setting_area = True
        self.auto_area_enabled = False
        self.detection_corners = []
        self.detection_area_set = False
        self.camera_canvas.delete("detection_point")
        self.camera_canvas.delete("detection_area")
        self.area_status.config(text="Click 4 points (0/4)", foreground="orange")
        self.log_message("Started manual area setting - click 4 points")
        
    def clear_detection_area(self):
        self.detection_corners = []
        self.detection_area_set = False
        self.setting_area = False
        self.auto_area_enabled = True
        self.camera_canvas.delete("detection_point")
        self.camera_canvas.delete("detection_area")
        self.area_status.config(text="Auto area ready", foreground="blue")
        self.log_message("Detection area cleared - auto mode enabled")
        
    def on_canvas_click(self, event):
        """Handle manual area setting clicks"""
        if not self.setting_area or len(self.detection_corners) >= 4:
            return
            
        x, y = event.x, event.y
        self.detection_corners.append((x, y))
        
        point_radius = 4
        self.camera_canvas.create_oval(x-point_radius, y-point_radius, 
                                     x+point_radius, y+point_radius,
                                     fill="red", outline="white", width=1,
                                     tags="detection_point")
        
        self.camera_canvas.create_text(x+8, y-8, text=str(len(self.detection_corners)),
                                     fill="yellow", font=("Arial", 10, "bold"),
                                     tags="detection_point")
        
        corner_count = len(self.detection_corners)
        self.area_status.config(text=f"Points ({corner_count}/4)", foreground="orange")
        
        self.log_message(f"Manual point {corner_count} set: ({x}, {y})")
        
        if corner_count > 1:
            prev_x, prev_y = self.detection_corners[-2]
            self.camera_canvas.create_line(prev_x, prev_y, x, y,
                                         fill="green", width=2,
                                         tags="detection_area")
        
        if corner_count == 4:
            first_x, first_y = self.detection_corners[0]
            self.camera_canvas.create_line(x, y, first_x, first_y,
                                         fill="green", width=2,
                                         tags="detection_area")
            
            self.setting_area = False
            self.detection_area_set = True
            self.area_status.config(text="Manual area set ✓", foreground="green")
            self.log_message("Manual detection area completed with 4 points")
            
    def start_conveyor(self):
        if not self.esp32_connected:
            messagebox.showerror("Error", "Please connect to ESP32 first")
            return
            
        if not self.detection_area_set:
            # Try auto-set if no area is set
            if not self.auto_set_detection_area():
                messagebox.showerror("Error", "Please set detection area first")
                return
        
        # RESET ALL STATE VARIABLES
        self.conveyor_running = True
        self.auto_detection_enabled = True
        self.manual_stop_requested = False
        self.is_scanning = False
        self.scan_timer_start = None
        self.products_stable_in_area = False
        self.motor_stopped = False
        self.last_motor_state = None
        
        self.conveyor_status.set("Starting - Motor HIGH")
        
        self.start_conveyor_btn.config(state=tk.DISABLED)
        self.stop_conveyor_btn.config(state=tk.NORMAL)
        
        # Start with motor HIGH
        self.send_motor_signal("HIGH")
        self.log_message("Conveyor belt started - Motor HIGH - Waiting for products")
        
    def stop_conveyor(self):
        self.conveyor_running = False
        self.auto_detection_enabled = False
        self.manual_stop_requested = True
        self.is_scanning = False
        self.scan_timer_start = None
        self.products_stable_in_area = False
        self.motor_stopped = False
        self.last_motor_state = None
        
        self.conveyor_status.set("Stopped")
        
        self.start_conveyor_btn.config(state=tk.NORMAL)
        self.stop_conveyor_btn.config(state=tk.DISABLED)
        
        self.send_motor_signal("LOW")
        self.log_message("Conveyor belt stopped manually")
        
    def point_in_polygon(self, point, polygon):
        """Check if point is inside polygon using ray casting algorithm"""
        x, y = point
        n = len(polygon)
        inside = False
        
        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
        
    def capture_photo(self, frame):
        """Capture and save current photo"""
        try:
            current_time = time.time()
            if current_time - self.last_capture_time < self.capture_cooldown:
                return False
                
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"product_scan_{timestamp}.jpg"
            filepath = os.path.join(self.photo_save_path, filename)
            
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            success = cv2.imwrite(filepath, frame_bgr)
            
            if success:
                self.last_capture_time = current_time
                self.log_message(f"Photo captured: {filename}")
                self.update_photo_count()
                return True
            else:
                self.log_message("Failed to save photo")
                return False
                
        except Exception as e:
            self.log_message(f"Photo capture error: {str(e)}")
            return False
            
    def manual_capture(self):
        """Manual photo capture"""
        if hasattr(self, 'current_frame') and self.current_frame is not None:
            if self.capture_photo(self.current_frame):
                messagebox.showinfo("Success", "Photo captured!")
            else:
                messagebox.showerror("Error", "Failed to capture photo")
        else:
            messagebox.showwarning("Warning", "No camera frame available")
            
    def set_photo_folder(self):
        """Select photo save folder"""
        folder = filedialog.askdirectory(initialdir=self.photo_save_path)
        if folder:
            self.photo_save_path = folder
            self.log_message(f"Photo save folder: {folder}")
            self.update_photo_count()
            
    def update_photo_count(self):
        """Update photo count display"""
        try:
            if os.path.exists(self.photo_save_path):
                photo_files = [f for f in os.listdir(self.photo_save_path) 
                             if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
                count = len(photo_files)
            else:
                count = 0
            self.photo_count_var.set(f"Photos: {count}")
        except:
            self.photo_count_var.set("Photos: Error")
        
    def update_camera(self):
        while self.camera_running and self.camera is not None:
            try:
                ret, frame = self.camera.read()
                if not ret:
                    break
                    
                frame = cv2.resize(frame, (self.canvas_width, self.canvas_height))
                
                labels_inside_area = []
                labels_outside_area = []
                Wrong_hole=[]
                Products=[]
                detection_text = "Detections:\n"
                
                if self.yolo_model:
                    results = self.yolo_model(frame, conf=self.threshold_var.get())
                    annotated_frame = results[0].plot()
                    
                    for r in results:
                        boxes = r.boxes
                        if boxes is not None:
                            for box in boxes:
                                cls = int(box.cls[0])
                                conf = float(box.conf[0])
                                class_name = self.yolo_model.names[cls]
                                
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                center_x = (x1 + x2) / 2
                                center_y = (y1 + y2) / 2
                                
                                detection_text += f"- {class_name}: {conf:.2f}\n"
                                
                                if self.detection_area_set:
                                    if self.point_in_polygon((center_x, center_y), self.detection_corners):
                                        if class_name == "R-Square" or class_name == "W-Square":
                                            labels_inside_area.append(class_name)
                                    else:
                                        labels_outside_area.append(class_name)
                    
                    frame_rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
                else:
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                self.current_frame = frame_rgb.copy()
                
                if labels_inside_area:
                    detection_text += f"\nINSIDE area: {', '.join(set(labels_inside_area))}"
                if labels_outside_area:
                    detection_text += f"\nOUTSIDE area: {', '.join(set(labels_outside_area))}"
                
                # ===== CONVEYOR CONTROL LOGIC - FIXED =====
                if self.conveyor_running and self.auto_detection_enabled and self.detection_area_set and not self.manual_stop_requested:
                    current_time = time.time()
                    self.scan_delay = self.scan_delay_var.get()
                    
                    # Check for products
                    square_inside = len(labels_inside_area) >= 1
                    square_outside = len(labels_outside_area) >= 1
                    has_products = len(labels_inside_area) > 0 or len(labels_outside_area) > 0
                    
                    if square_inside and not self.is_scanning and not self.products_stable_in_area:
                        # CASE 1: Products enter detection area - STOP MOTOR and START SCAN
                        self.is_scanning = True
                        self.scan_timer_start = current_time
                        self.motor_stopped = True
                        
                        # Send STOP signal
                        self.send_motor_signal("LOW")
                        
                        self.detection_status.set("Products detected - Starting scan")
                        self.conveyor_status.set(f"STOPPED - Scanning... {self.scan_delay:.1f}s")
                        self.log_message(f"Products in detection area - Motor STOPPED - Starting {self.scan_delay}s scan")
                        if self.detection_area_set:
                            if self.point_in_polygon((center_x, center_y), self.detection_corners):
                                Products.append(class_name) 
                    elif self.is_scanning and square_inside:
                        # CASE 2: Currently scanning - update remaining time
                        if self.scan_timer_start and (current_time - self.scan_timer_start) >= self.scan_delay:
                            # Scan complete
                            self.is_scanning = False
                            self.products_stable_in_area = True
                            # Capture photo if enabled
                            if self.capture_enabled_var.get():
                                if self.capture_photo(self.current_frame):
                                    detection_text += f"\n*** PHOTO CAPTURED ***"
                                    self.log_message("Scan completed - Photo captured successfully")
                                else:
                                    self.log_message("Scan completed - Photo capture failed")

                            self.detection_status.set("Scan completed - Products ready for removal")
                            self.detection_status.set("Checking products")
                            Wrong_hole=list(set(W_products)&set(Products))    
                            self.conveyor_status.set("SCAN COMPLETED - Remove products to continue")
                            
                        else:
                            # Still scanning - show remaining time
                            remaining_time = self.scan_delay - (current_time - self.scan_timer_start)
                            self.conveyor_status.set(f"SCANNING... {remaining_time:.1f}s remaining")
                            self.detection_status.set(f"Scanning products... {remaining_time:.1f}s")
                            
                    elif self.products_stable_in_area and square_outside:
                        # CASE 3: Products scanned and moved outside detection area - RESTART MOTOR
                        self.products_stable_in_area = False
                        self.is_scanning = False
                        self.scan_timer_start = None
                        self.motor_stopped = False
                        
                        # Restart motor
                        self.send_motor_signal("HIGH")
                        
                        self.detection_status.set("Products removed - Motor restarted")
                        self.conveyor_status.set("RUNNING - Ready for next products")
                        self.log_message("Products removed from detection area - Motor RESTARTED")
                        
                    elif not has_products and not self.is_scanning and not self.products_stable_in_area:
                        # CASE 4: No products - MOTOR RUNNING
                        if self.motor_stopped:
                            self.motor_stopped = False
                            self.send_motor_signal("HIGH")
                            self.log_message("No products detected - Motor RUNNING")
                        
                        self.detection_status.set("No products - Waiting")
                        self.conveyor_status.set("RUNNING - Waiting for products")
                        
                    elif square_outside and not self.is_scanning and not self.products_stable_in_area:
                        # CASE 5: Products outside detection area - MOTOR RUNNING to move them in
                        if self.motor_stopped:
                            self.motor_stopped = False
                            self.send_motor_signal("HIGH")
                            self.log_message("Products outside detection area - Motor RUNNING to move products")
                        
                        self.detection_status.set("Products outside area - Moving to detection zone")
                        self.conveyor_status.set("RUNNING - Moving products to scan area")
                if Wrong_hole:
                    self.log_message("hư cmnr")
                    self.quick_servo(180)
                else:
                    self.log_message("bơ phẹt")
                self.root.after(0, self.update_camera_display, frame_rgb)
                self.root.after(0, self.update_detection_info, detection_text)
                
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                self.log_message(f"Camera error: {e}")
                break
                
    def update_camera_display(self, frame_rgb):
        try:
            image = Image.fromarray(frame_rgb)
            photo = ImageTk.PhotoImage(image)
            
            self.camera_canvas.delete("camera_image")
            self.camera_canvas.create_image(0, 0, anchor=tk.NW, image=photo, tags="camera_image")
            
            self.camera_canvas.tag_raise("detection_area")
            self.camera_canvas.tag_raise("detection_point")
            
            self.camera_canvas.image = photo
        except Exception as e:
            self.log_message(f"Display error: {e}")
        
    def update_detection_info(self, text):
        try:
            self.detection_info.delete(1.0, tk.END)
            self.detection_info.insert(tk.END, text)
        except Exception as e:
            self.log_message(f"Detection info update error: {e}")
        
    def connect_esp32(self):
        ip = self.ip_entry.get().strip()
        if not ip:
            messagebox.showerror("Error", "Please enter ESP32 IP address")
            return
            
        try:
            response = requests.get(f"http://{ip}", timeout=5)
            if response.status_code == 200:
                self.esp32_ip = ip
                self.esp32_connected = True
                self.connection_status.set("Connected")
                
                self.connect_btn.config(state=tk.DISABLED)
                self.disconnect_btn.config(state=tk.NORMAL)
                
                self.log_message(f"Connected to ESP32 at {ip}")
                messagebox.showinfo("Success", "Connected to ESP32!")
            else:
                raise Exception("Invalid response")
                
        except Exception as e:
            messagebox.showerror("Connection Error", 
                               f"Could not connect to ESP32 at {ip}\nError: {str(e)}")
            self.log_message(f"Connection failed: {str(e)}")
            
    def disconnect_esp32(self):
        self.esp32_connected = False
        self.esp32_ip = ""
        self.connection_status.set("Disconnected")
        
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        
        self.log_message("Disconnected from ESP32")
        
    def send_motor_signal(self, signal):
        """Send motor signal with thread safety and state tracking"""
        if not self.esp32_connected:
            return
        
        # Avoid sending duplicate signals
        if self.last_motor_state == signal:
            return
            
        def send_request():
            try:
                with self.request_lock:
                    url = f"http://{self.esp32_ip}/motorControl?signal={signal}"
                    response = requests.get(url, timeout=3)
                    
                    if response.status_code == 200:
                        self.last_motor_state = signal
                        status = "RUNNING" if signal == "HIGH" else "STOPPED"
                        self.log_message(f"Motor: {signal} -> {status}")
                    else:
                        self.log_message(f"Motor control error: HTTP {response.status_code}")
                        
            except requests.exceptions.Timeout:
                self.log_message("Motor control timeout - check ESP32 connection")
            except Exception as e:
                self.log_message(f"Motor control error: {str(e)}")
        
        threading.Thread(target=send_request, daemon=True).start()
            
    def send_servo_angle(self):
        """Send servo angle with thread safety"""
        if not self.esp32_connected:
            messagebox.showerror("Error", "Not connected to ESP32")
            return
            
        def send_request():
            try:
                angle = int(float(self.current_servo_angle.get()))
                with self.request_lock:
                    url = f"http://{self.esp32_ip}/setServo?angle={angle}"
                    response = requests.get(url, timeout=3)
                    
                    if response.status_code == 200:
                        self.log_message(f"Servo angle set to {angle}°")
                    else:
                        self.log_message(f"Servo error: HTTP {response.status_code}")
                        
            except requests.exceptions.Timeout:
                self.log_message("Servo control timeout - check ESP32 connection")
            except Exception as e:
                self.log_message(f"Servo error: {str(e)}")
                
        threading.Thread(target=send_request, daemon=True).start()
            
    def quick_servo(self, angle):
        self.current_servo_angle.set(str(angle))
        self.send_servo_angle()
            
    def log_message(self, message):
        try:
            current_time = time.strftime("%H:%M:%S")
            log_entry = f"[{current_time}] {message}\n"
            
            self.log_text.insert(tk.END, log_entry)
            self.log_text.see(tk.END)
            
            # Keep only last 50 lines to save memory
            lines = self.log_text.get("1.0", tk.END).split('\n')
            if len(lines) > 50:
                self.log_text.delete("1.0", "2.0")
        except Exception as e:
            print(f"Log error: {e}")
            
    def on_closing(self):
        try:
            self.camera_running = False
            if self.camera:
                self.camera.release()
            cv2.destroyAllWindows()
        except:
            pass
        finally:
            self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ESP32ConveyorControlGUI(root)
    
    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    root.mainloop()