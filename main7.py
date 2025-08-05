import tkinter as tk
from tkinter import ttk, messagebox
import cv2
from PIL import Image, ImageTk
import requests
import threading
import time
from ultralytics import YOLO
import numpy as np

class ESP32ConveyorControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Conveyor Belt Control with YOLO Detection")
        self.root.geometry("1500x900")
        
        # ESP32 Configuration
        self.esp32_ip = ""
        self.esp32_connected = False
        
        # Camera and YOLO Configuration
        self.camera = None
        self.yolo_model = None
        self.camera_running = False
        
        # Detection area configuration
        self.detection_corners = []  # Will store 4 corner points
        self.detection_area_set = False
        self.setting_area = False
        
        # Conveyor belt control
        self.conveyor_running = False
        self.auto_detection_enabled = False
        
        # Detection parameters
        self.detection_threshold = 0.5
        self.labels_outside_area = True  # True when labels are outside the square
        
        # Current servo angle
        self.current_servo_angle = tk.StringVar(value="90")
        
        # Status variables
        self.connection_status = tk.StringVar(value="Disconnected")
        self.conveyor_status = tk.StringVar(value="Stopped")
        self.detection_status = tk.StringVar(value="No detections")
        
        self.setup_gui()
        self.load_yolo_model()
        
    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left frame for camera
        left_frame = ttk.LabelFrame(main_frame, text="YOLO Camera Detection", padding=10)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # Camera display
        self.camera_canvas = tk.Canvas(left_frame, bg="black", width=800, height=600)
        self.camera_canvas.pack(pady=10)
        
        # Bind mouse click event for setting detection area
        self.camera_canvas.bind("<Button-1>", self.on_canvas_click)
        
        # Camera controls
        camera_controls = ttk.Frame(left_frame)
        camera_controls.pack(pady=10)
        
        self.start_camera_btn = tk.Button(camera_controls, text="Start Camera", 
                                          command=self.start_camera,
                                          bg="#4CAF50", fg="white",
                                          font=("Arial", 12, "bold"),
                                          relief=tk.RAISED, borderwidth=3,
                                          padx=15, pady=8)
        self.start_camera_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_camera_btn = tk.Button(camera_controls, text="Stop Camera", 
                                         command=self.stop_camera, state=tk.DISABLED,
                                         bg="#f44336", fg="white",
                                         font=("Arial", 12, "bold"),
                                         relief=tk.RAISED, borderwidth=3,
                                         padx=15, pady=8)
        self.stop_camera_btn.pack(side=tk.LEFT, padx=5)
        
        # Detection area setup
        area_frame = ttk.LabelFrame(left_frame, text="Detection Area Setup", padding=10)
        area_frame.pack(fill=tk.X, pady=10)
        
        area_instructions = ttk.Label(area_frame, 
                                     text="Click 4 points on camera to create detection square",
                                     font=("Arial", 10, "italic"))
        area_instructions.pack()
        
        area_buttons = ttk.Frame(area_frame)
        area_buttons.pack(pady=5)
        
        self.set_area_btn = tk.Button(area_buttons, text="Set Detection Area", 
                                     command=self.start_setting_area,
                                     bg="#FF9800", fg="white",
                                     font=("Arial", 11, "bold"),
                                     relief=tk.RAISED, borderwidth=2,
                                     padx=10, pady=5)
        self.set_area_btn.pack(side=tk.LEFT, padx=5)
        
        self.clear_area_btn = tk.Button(area_buttons, text="Clear Area", 
                                       command=self.clear_detection_area,
                                       bg="#9E9E9E", fg="white",
                                       font=("Arial", 11, "bold"),
                                       relief=tk.RAISED, borderwidth=2,
                                       padx=10, pady=5)
        self.clear_area_btn.pack(side=tk.LEFT, padx=5)
        
        self.area_status = ttk.Label(area_frame, text="No detection area set", 
                                    foreground="red")
        self.area_status.pack(pady=5)
        
        # Conveyor belt control
        conveyor_frame = ttk.LabelFrame(left_frame, text="Conveyor Belt Control", padding=10)
        conveyor_frame.pack(fill=tk.X, pady=10)
        
        conveyor_buttons = ttk.Frame(conveyor_frame)
        conveyor_buttons.pack(pady=10)
        
        self.start_conveyor_btn = tk.Button(conveyor_buttons, text="START CONVEYOR", 
                                           command=self.start_conveyor,
                                           bg="#4CAF50", fg="white",
                                           font=("Arial", 14, "bold"),
                                           relief=tk.RAISED, borderwidth=4,
                                           padx=20, pady=10)
        self.start_conveyor_btn.pack(side=tk.LEFT, padx=10)
        
        self.stop_conveyor_btn = tk.Button(conveyor_buttons, text="STOP CONVEYOR", 
                                          command=self.stop_conveyor,
                                          bg="#f44336", fg="white",
                                          font=("Arial", 14, "bold"),
                                          relief=tk.RAISED, borderwidth=4,
                                          padx=20, pady=10, state=tk.DISABLED)
        self.stop_conveyor_btn.pack(side=tk.LEFT, padx=10)
        
        # Status display for conveyor
        conveyor_status_frame = ttk.Frame(conveyor_frame)
        conveyor_status_frame.pack(pady=10)
        
        ttk.Label(conveyor_status_frame, text="Conveyor Status:", 
                 font=("Arial", 12, "bold")).pack(side=tk.LEFT)
        conveyor_status_display = ttk.Label(conveyor_status_frame, textvariable=self.conveyor_status,
                                           font=("Arial", 12, "bold"), foreground="blue")
        conveyor_status_display.pack(side=tk.LEFT, padx=10)
        
        # Detection info
        detection_frame = ttk.LabelFrame(left_frame, text="Detection Information", padding=10)
        detection_frame.pack(fill=tk.X, pady=10)
        
        self.detection_info = tk.Text(detection_frame, height=6, width=70)
        self.detection_info.pack(pady=5, fill=tk.X)
        
        # Right frame for ESP32 controls
        right_frame = ttk.LabelFrame(main_frame, text="ESP32 Control Panel", padding=10)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        
        # Connection section
        conn_frame = ttk.LabelFrame(right_frame, text="ESP32 Connection", padding=10)
        conn_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(conn_frame, text="ESP32 IP Address:").pack(anchor=tk.W)
        self.ip_entry = ttk.Entry(conn_frame, width=25)
        self.ip_entry.pack(pady=5, fill=tk.X)
        self.ip_entry.insert(0, "192.168.1.100")  # Default IP
        
        conn_buttons = ttk.Frame(conn_frame)
        conn_buttons.pack(fill=tk.X, pady=5)
        
        self.connect_btn = tk.Button(conn_buttons, text="Connect", 
                                     command=self.connect_esp32,
                                     bg="#2196F3", fg="white",
                                     font=("Arial", 11, "bold"),
                                     relief=tk.RAISED, borderwidth=2,
                                     padx=10, pady=5)
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        self.disconnect_btn = tk.Button(conn_buttons, text="Disconnect", 
                                        command=self.disconnect_esp32, state=tk.DISABLED,
                                        bg="#FF9800", fg="white",
                                        font=("Arial", 11, "bold"),
                                        relief=tk.RAISED, borderwidth=2,
                                        padx=10, pady=5)
        self.disconnect_btn.pack(side=tk.LEFT)
        
        # Status display
        status_frame = ttk.LabelFrame(right_frame, text="System Status", padding=10)
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(status_frame, text="ESP32 Connection:").pack(anchor=tk.W)
        connection_display = ttk.Label(status_frame, textvariable=self.connection_status, 
                                      foreground="red", font=("Arial", 10, "bold"))
        connection_display.pack(anchor=tk.W, padx=10)
        
        ttk.Label(status_frame, text="Detection Status:").pack(anchor=tk.W, pady=(10, 0))
        detection_display = ttk.Label(status_frame, textvariable=self.detection_status,
                                     foreground="green", font=("Arial", 10, "bold"))
        detection_display.pack(anchor=tk.W, padx=10)
        
        # Servo Control section
        servo_frame = ttk.LabelFrame(right_frame, text="Servo Control", padding=10)
        servo_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(servo_frame, text="Servo Angle (0-180°):").pack(anchor=tk.W)
        self.servo_scale = ttk.Scale(servo_frame, from_=0, to=180, orient=tk.HORIZONTAL,
                                    variable=self.current_servo_angle, length=200)
        self.servo_scale.pack(fill=tk.X, pady=5)
        
        servo_display = ttk.Label(servo_frame, textvariable=self.current_servo_angle)
        servo_display.pack()
        
        ttk.Button(servo_frame, text="Set Servo Angle", 
                  command=self.send_servo_angle).pack(pady=10)
        
        # Quick servo positions
        quick_servo_frame = ttk.Frame(servo_frame)
        quick_servo_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(quick_servo_frame, text="0°", 
                  command=lambda: self.quick_servo(0)).pack(side=tk.LEFT, padx=2)
        ttk.Button(quick_servo_frame, text="90°", 
                  command=lambda: self.quick_servo(90)).pack(side=tk.LEFT, padx=2)
        ttk.Button(quick_servo_frame, text="180°", 
                  command=lambda: self.quick_servo(180)).pack(side=tk.LEFT, padx=2)
        
        # Detection Configuration
        config_frame = ttk.LabelFrame(right_frame, text="Detection Config", padding=10)
        config_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(config_frame, text="Detection Threshold:").pack(anchor=tk.W)
        self.threshold_var = tk.DoubleVar(value=0.5)
        threshold_scale = ttk.Scale(config_frame, from_=0.1, to=0.9, orient=tk.HORIZONTAL,
                                   variable=self.threshold_var, length=200)
        threshold_scale.pack(fill=tk.X, pady=2)
        
        threshold_display = ttk.Label(config_frame, textvariable=self.threshold_var)
        threshold_display.pack()
        
        # Manual Motor Control (for testing)
        manual_frame = ttk.LabelFrame(right_frame, text="Manual Motor Test", padding=10)
        manual_frame.pack(fill=tk.X, pady=(0, 10))
        
        manual_buttons = ttk.Frame(manual_frame)
        manual_buttons.pack(fill=tk.X, pady=5)
        
        ttk.Button(manual_buttons, text="Motor HIGH", 
                  command=lambda: self.send_motor_signal("HIGH")).pack(side=tk.LEFT, padx=2)
        ttk.Button(manual_buttons, text="Motor LOW", 
                  command=lambda: self.send_motor_signal("LOW")).pack(side=tk.LEFT, padx=2)
        
        # System log
        log_frame = ttk.LabelFrame(right_frame, text="System Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_text = tk.Text(log_frame, height=10, width=35, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        log_scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, 
                                     command=self.log_text.yview)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=log_scrollbar.set)
        
    def load_yolo_model(self):
        try:
            self.yolo_model = YOLO('best.pt')  # Sử dụng model custom của bạn
            self.log_message("YOLO model loaded successfully!")
            self.update_detection_info("YOLO model ready. Waiting for camera...")
        except Exception as e:
            error_msg = f"Error loading YOLO model: {str(e)}"
            self.log_message(error_msg)
            self.update_detection_info(error_msg)
            
    def start_camera(self):
        try:
            self.camera = cv2.VideoCapture(1)  # Thay đổi index nếu cần
            if not self.camera.isOpened():
                messagebox.showerror("Error", "Cannot open camera")
                return
                
            self.camera_running = True
            self.start_camera_btn.config(state=tk.DISABLED)
            self.stop_camera_btn.config(state=tk.NORMAL)
            
            # Start camera thread
            self.camera_thread = threading.Thread(target=self.update_camera, daemon=True)
            self.camera_thread.start()
            
            self.log_message("Camera started successfully")
            
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
        self.log_message("Camera stopped")
        
    def start_setting_area(self):
        if not self.camera_running:
            messagebox.showwarning("Warning", "Please start camera first")
            return
            
        self.setting_area = True
        self.detection_corners = []
        self.detection_area_set = False
        self.area_status.config(text="Click 4 points on camera (0/4)", foreground="orange")
        self.log_message("Started setting detection area - click 4 points")
        
    def clear_detection_area(self):
        self.detection_corners = []
        self.detection_area_set = False
        self.setting_area = False
        self.area_status.config(text="No detection area set", foreground="red")
        self.log_message("Detection area cleared")
        
    def on_canvas_click(self, event):
        if not self.setting_area or len(self.detection_corners) >= 4:
            return
            
        # Add clicked point to corners list
        x, y = event.x, event.y
        self.detection_corners.append((x, y))
        
        corner_count = len(self.detection_corners)
        self.area_status.config(text=f"Click 4 points on camera ({corner_count}/4)", 
                               foreground="orange")
        
        self.log_message(f"Point {corner_count} set: ({x}, {y})")
        
        # If we have 4 points, complete the area setup
        if corner_count == 4:
            self.setting_area = False
            self.detection_area_set = True
            self.area_status.config(text="Detection area set (4 points)", foreground="green")
            self.log_message("Detection area completed with 4 points")
            
    def start_conveyor(self):
        if not self.esp32_connected:
            messagebox.showerror("Error", "Please connect to ESP32 first")
            return
            
        if not self.detection_area_set:
            messagebox.showerror("Error", "Please set detection area first")
            return
            
        self.conveyor_running = True
        self.auto_detection_enabled = True
        self.conveyor_status.set("Running - Waiting for products")
        
        self.start_conveyor_btn.config(state=tk.DISABLED)
        self.stop_conveyor_btn.config(state=tk.NORMAL)
        
        # Send initial HIGH signal to start motor
        self.send_motor_signal("HIGH")
        self.log_message("Conveyor belt started - waiting for products")
        
    def stop_conveyor(self):
        self.conveyor_running = False
        self.auto_detection_enabled = False
        self.conveyor_status.set("Stopped")
        
        self.start_conveyor_btn.config(state=tk.NORMAL)
        self.stop_conveyor_btn.config(state=tk.DISABLED)
        
        # Send LOW signal to stop motor
        self.send_motor_signal("LOW")
        self.log_message("Conveyor belt stopped")
        
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
        
    def update_camera(self):
        while self.camera_running:
            try:
                ret, frame = self.camera.read()
                if not ret:
                    break
                    
                # Resize frame
                frame = cv2.resize(frame, (800, 600))
                
                # Run YOLO detection
                labels_inside_area = []
                labels_outside_area = []
                detection_text = "Detections:\n"
                
                if self.yolo_model:
                    results = self.yolo_model(frame, conf=self.threshold_var.get())
                    
                    # Draw detection results
                    annotated_frame = results[0].plot()
                    
                    # Check detections
                    for r in results:
                        boxes = r.boxes
                        if boxes is not None:
                            for box in boxes:
                                cls = int(box.cls[0])
                                conf = float(box.conf[0])
                                class_name = self.yolo_model.names[cls]
                                
                                # Get box center coordinates
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                center_x = (x1 + x2) / 2
                                center_y = (y1 + y2) / 2
                                
                                detection_text += f"- {class_name}: {conf:.2f}\n"
                                
                                # Check if detection center is inside the polygon
                                if self.detection_area_set:
                                    if self.point_in_polygon((center_x, center_y), self.detection_corners):
                                        labels_inside_area.append(class_name)
                                    else:
                                        labels_outside_area.append(class_name)
                    
                    # Draw detection area if set
                    if self.detection_area_set and len(self.detection_corners) == 4:
                        # Draw polygon
                        points = np.array(self.detection_corners, np.int32)
                        cv2.polylines(annotated_frame, [points], True, (0, 255, 0), 3)
                        cv2.fillPoly(annotated_frame, [points], (0, 255, 0, 30))  # Semi-transparent fill
                        
                        # Draw corner points
                        for i, (x, y) in enumerate(self.detection_corners):
                            cv2.circle(annotated_frame, (x, y), 8, (255, 0, 0), -1)
                            cv2.putText(annotated_frame, str(i+1), (x-5, y-10),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        
                        cv2.putText(annotated_frame, "Detection Area", 
                                  (self.detection_corners[0][0], self.detection_corners[0][1] - 20),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    
                    frame_rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
                else:
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Update detection info
                if labels_inside_area:
                    detection_text += f"\nLabels INSIDE area: {', '.join(set(labels_inside_area))}"
                if labels_outside_area:
                    detection_text += f"\nLabels OUTSIDE area: {', '.join(set(labels_outside_area))}"
                
                # Auto control logic for conveyor
                if self.conveyor_running and self.auto_detection_enabled and self.detection_area_set:
                    # Logic: If ANY labels exist outside the detection area, keep conveyor running
                    # If NO labels outside area (all products are inside), stop conveyor
                    
                    current_labels_outside = len(labels_outside_area) > 0
                    
                    if current_labels_outside != self.labels_outside_area:
                        self.labels_outside_area = current_labels_outside
                        
                        if self.labels_outside_area:
                            # There are labels outside area -> Keep conveyor running (HIGH)
                            self.send_motor_signal("HIGH")
                            self.detection_status.set("Products outside area - Running")
                            self.conveyor_status.set("Running - Products moving")
                        else:
                            # No labels outside area -> All products in area, stop conveyor (LOW)
                            self.send_motor_signal("LOW")
                            self.detection_status.set("All products in area - Stopped")
                            self.conveyor_status.set("Stopped - Products in area")
                
                # Update GUI in main thread
                self.root.after(0, self.update_camera_display, frame_rgb)
                self.root.after(0, self.update_detection_info, detection_text)
                
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                self.log_message(f"Camera error: {e}")
                break
                
    def update_camera_display(self, frame_rgb):
        image = Image.fromarray(frame_rgb)
        photo = ImageTk.PhotoImage(image)
        self.camera_canvas.delete("all")
        self.camera_canvas.create_image(0, 0, anchor=tk.NW, image=photo)
        self.camera_canvas.image = photo
        
    def update_detection_info(self, text):
        self.detection_info.delete(1.0, tk.END)
        self.detection_info.insert(tk.END, text)
        
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
        if not self.esp32_connected:
            self.log_message("Error: Not connected to ESP32")
            return
            
        try:
            url = f"http://{self.esp32_ip}/motorControl?signal={signal}"
            response = requests.get(url, timeout=5)
            
            if response.status_code == 200:
                status = "Running" if signal == "HIGH" else "Stopped"
                self.log_message(f"Motor signal sent: {signal} -> {status}")
            else:
                self.log_message(f"Error: HTTP {response.status_code}")
                
        except Exception as e:
            self.log_message(f"Error sending motor signal: {str(e)}")
            
    def send_servo_angle(self):
        if not self.esp32_connected:
            messagebox.showerror("Error", "Not connected to ESP32")
            return
            
        try:
            angle = int(float(self.current_servo_angle.get()))
            url = f"http://{self.esp32_ip}/setServo?angle={angle}"
            
            response = requests.get(url, timeout=5)
            
            if response.status_code == 200:
                self.log_message(f"Servo angle set to {angle}°")
            else:
                self.log_message(f"Error: HTTP {response.status_code}")
                
        except Exception as e:
            self.log_message(f"Error setting servo: {str(e)}")
            messagebox.showerror("Error", f"Failed to set servo angle: {str(e)}")
            
    def quick_servo(self, angle):
        self.current_servo_angle.set(str(angle))
        self.send_servo_angle()
            
    def log_message(self, message):
        current_time = time.strftime("%H:%M:%S")
        log_entry = f"[{current_time}] {message}\n"
        
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
        
        # Keep only last 100 lines
        lines = self.log_text.get("1.0", tk.END).split('\n')
        if len(lines) > 100:
            self.log_text.delete("1.0", "2.0")
            
    def on_closing(self):
        self.stop_camera()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ESP32ConveyorControlGUI(root)
    
    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    root.mainloop()