import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import cv2
from PIL import Image, ImageTk
import requests
import threading
import time
from ultralytics import YOLO
import numpy as np

class ESP32ControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Motor & Servo Control with YOLO Detection")
        self.root.geometry("1500x1000")
    # ... (các biến khác)
        
        # ESP32 Configuration
        self.esp32_ip = ""
        self.esp32_connected = False
        
        # Camera and YOLO Configuration
        self.camera = None
        self.yolo_model = None
        self.camera_running = False
        
        # Current values
        self.current_pwm = tk.StringVar(value="255")
        self.current_delay = tk.StringVar(value="2000")
        self.current_angle = tk.StringVar(value="0")
        self.current_servo_speed = tk.StringVar(value="50")
        
        # Status variables
        self.connection_status = tk.StringVar(value="Disconnected")
        self.response_text = tk.StringVar(value="No response yet")
        
        self.setup_gui()
        self.load_yolo_model()
        
    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left frame for camera
        left_frame = ttk.LabelFrame(main_frame, text="YOLO Camera Detection", padding=10)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # Camera display label
        self.camera_canvas = tk.Canvas(left_frame, bg="black", width=800, height=600)
        self.camera_canvas.pack(pady=10)
        
        # Camera controls
        camera_controls = ttk.Frame(left_frame)
        camera_controls.pack(pady=5)
        
        self.start_camera_btn = tk.Button(camera_controls, text="Start", 
                                          command=self.start_camera,
                                          bg="#4CAF50",
                                          fg="red",
                                          font=("Arial", 12, "bold"),
                                          relief=tk.RAISED,
                                          borderwidth=4,
                                          padx=10, pady=5)
        self.start_camera_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_camera_btn = tk.Button(camera_controls, text="End", 
                                         command=self.stop_camera, state=tk.DISABLED,
                                         bg="#4CAF50",
                                         fg="yellow",
                                         font=("Arial", 12, "bold"),
                                         relief=tk.RAISED,
                                         borderwidth=4,
                                         padx=10, pady=5)
        self.stop_camera_btn.pack(side=tk.LEFT, padx=5)
        
        # Detection info
        self.detection_info = tk.Text(left_frame, height=8, width=70)
        self.detection_info.pack(pady=10, fill=tk.X)
        
        # Right frame for ESP32 controls
        right_frame = ttk.LabelFrame(main_frame, text="ESP32 Control Panel", padding=10)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(5, 0))
        
        # Connection section
        conn_frame = ttk.LabelFrame(right_frame, text="Connection", padding=10)
        conn_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(conn_frame, text="ESP32 IP Address:").pack(anchor=tk.W)
        self.ip_entry = ttk.Entry(conn_frame, width=20)
        self.ip_entry.pack(pady=5, fill=tk.X)
        
        conn_buttons = ttk.Frame(conn_frame)
        conn_buttons.pack(fill=tk.X, pady=5)
        
        self.connect_btn = tk.Button(conn_buttons, text="Connect", 
                                     command=self.connect_esp32,
                                     bg="#4CAF50",
                                     fg="yellow",
                                     font=("Arial", 12, "bold"),
                                     relief=tk.RAISED,
                                     borderwidth=4,
                                     padx=10, pady=5)
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        self.disconnect_btn = tk.Button(conn_buttons, text="Disconnect", 
                                        command=self.disconnect_esp32, state=tk.DISABLED,
                                        bg="#4CAF50",
                                        fg="yellow",
                                        font=("Arial", 12, "bold"),
                                        relief=tk.RAISED,
                                        borderwidth=4,
                                        padx=10, pady=5)
        self.disconnect_btn.pack(side=tk.LEFT)
        
        # Status label
        status_label = ttk.Label(conn_frame, textvariable=self.connection_status, 
                                foreground="red")
        status_label.pack(pady=5)
        
        # Motor Control section
        motor_frame = ttk.LabelFrame(right_frame, text="Motor Control", padding=10)
        motor_frame.pack(fill=tk.X, pady=(0, 10))
        
        # PWM Speed
        ttk.Label(motor_frame, text="PWM Speed (0-255):").pack(anchor=tk.W)
        self.pwm_scale = ttk.Scale(motor_frame, from_=0, to=255, orient=tk.HORIZONTAL,
                                  variable=self.current_pwm, length=200)
        self.pwm_scale.pack(fill=tk.X, pady=2)
        pwm_display = ttk.Label(motor_frame, textvariable=self.current_pwm)
        pwm_display.pack()
        
        # Delay Time
        ttk.Label(motor_frame, text="Run/Stop Delay (ms):").pack(anchor=tk.W, pady=(10, 0))
        self.delay_scale = ttk.Scale(motor_frame, from_=100, to=5000, orient=tk.HORIZONTAL,
                                    variable=self.current_delay, length=200)
        self.delay_scale.pack(fill=tk.X, pady=2)
        delay_display = ttk.Label(motor_frame, textvariable=self.current_delay)
        delay_display.pack()
        
        # Servo Control section
        servo_frame = ttk.LabelFrame(right_frame, text="Servo Control", padding=10)
        servo_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Servo Angle
        ttk.Label(servo_frame, text="Servo Angle (0-180°):").pack(anchor=tk.W)
        self.angle_scale = ttk.Scale(servo_frame, from_=0, to=180, orient=tk.HORIZONTAL,
                                    variable=self.current_angle, length=200)
        self.angle_scale.pack(fill=tk.X, pady=2)
        angle_display = ttk.Label(servo_frame, textvariable=self.current_angle)
        angle_display.pack()
        
        # Servo Speed
        ttk.Label(servo_frame, text="Servo Speed (10-500 ms/step):").pack(anchor=tk.W, pady=(10, 0))
        self.servo_speed_scale = ttk.Scale(servo_frame, from_=10, to=500, orient=tk.HORIZONTAL,
                                          variable=self.current_servo_speed, length=200)
        self.servo_speed_scale.pack(fill=tk.X, pady=2)
        speed_display = ttk.Label(servo_frame, textvariable=self.current_servo_speed)
        speed_display.pack()
        
        # Send Command section
        command_frame = ttk.LabelFrame(right_frame, text="Send Command", padding=10)
        command_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.send_btn = ttk.Button(command_frame, text="Send Parameters", 
                                  command=self.send_parameters, state=tk.DISABLED)
        self.send_btn.pack(pady=10)
        
        # Quick Actions
        quick_frame = ttk.LabelFrame(right_frame, text="Quick Actions", padding=10)
        quick_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(quick_frame, text="Stop Motor", 
                  command=lambda: self.quick_send(pwm=0)).pack(fill=tk.X, pady=2)
        ttk.Button(quick_frame, text="Full Speed", 
                  command=lambda: self.quick_send(pwm=255)).pack(fill=tk.X, pady=2)
        ttk.Button(quick_frame, text="Servo 0°", 
                  command=lambda: self.quick_send(angle=0)).pack(fill=tk.X, pady=2)
        ttk.Button(quick_frame, text="Servo 90°", 
                  command=lambda: self.quick_send(angle=90)).pack(fill=tk.X, pady=2)
        ttk.Button(quick_frame, text="Servo 180°", 
                  command=lambda: self.quick_send(angle=180)).pack(fill=tk.X, pady=2)
        
        # Response section
        response_frame = ttk.LabelFrame(right_frame, text="ESP32 Response", padding=10)
        response_frame.pack(fill=tk.BOTH, expand=True)
        
        self.response_text_widget = tk.Text(response_frame, height=6, width=30, wrap=tk.WORD)
        self.response_text_widget.pack(fill=tk.BOTH, expand=True)
        
        # Scrollbar for response text
        scrollbar = ttk.Scrollbar(response_frame, orient=tk.VERTICAL, 
                                 command=self.response_text_widget.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.response_text_widget.config(yscrollcommand=scrollbar.set)
        
    def load_yolo_model(self):
        try:
            # Load YOLO model (you can change to yolov8n.pt, yolov8s.pt, etc.)
            self.yolo_model = YOLO('best.pt')  # Downloads automatically if not present
            self.update_detection_info("YOLO model loaded successfully!")
        except Exception as e:
            self.update_detection_info(f"Error loading YOLO model: {str(e)}")
            
    def start_camera(self):
        try:
            self.camera = cv2.VideoCapture(1)  # Use default camera
            if not self.camera.isOpened():
                messagebox.showerror("Error", "Cannot open camera")
                return
            self.camera_running = True
            self.start_camera_btn.config(state=tk.DISABLED)
            self.stop_camera_btn.config(state=tk.NORMAL)
            
            # Start camera thread
            self.camera_thread = threading.Thread(target=self.update_camera, daemon=True)
            self.camera_thread.start()
            
        except Exception as e:
            messagebox.showerror("Error", f"Error starting camera: {str(e)}")
            
    def stop_camera(self):
        self.camera_running = False
        if self.camera:
            self.camera.release()
        self.camera_canvas.delete("all")
        self.start_camera_btn.config(state=tk.NORMAL)
        self.stop_camera_btn.config(state=tk.DISABLED)
       
        
    def update_camera(self):
        while self.camera_running:
            try:
                ret, frame = self.camera.read()
                if not ret:
                    break
                    
                # Run YOLO detection
                if self.yolo_model:
                    results = self.yolo_model(frame)
                    
                    # Draw detection results
                    annotated_frame = results[0].plot()
                    
                    # Update detection info
                    detection_text = "Detections:\n"
                    for r in results:
                        boxes = r.boxes
                        if boxes is not None:
                            for box in boxes:
                                cls = int(box.cls[0])
                                conf = float(box.conf[0])
                                class_name = self.yolo_model.names[cls]
                                detection_text += f"- {class_name}: {conf:.2f}\n"
                    
                    self.root.after(0, self.update_detection_info, detection_text)
                    
                    # Convert to RGB for display
                    frame_rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
                else:
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Resize frame to fit display
                frame_rgb = cv2.resize(frame_rgb, (800, 600))
                
                # Convert to PhotoImage
                image = Image.fromarray(frame_rgb)
                photo = ImageTk.PhotoImage(image)
                
                # Update GUI in main thread
                self.root.after(0, self.update_camera_display, photo)
                
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                print(f"Camera error: {e}")
                break
                
    def update_camera_display(self, photo):
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
            
        # Test connection
        try:
            response = requests.get(f"http://{ip}", timeout=5)
            if response.status_code == 200:
                self.esp32_ip = ip
                self.esp32_connected = True
                self.connection_status.set("Connected")
                self.connection_status.config(foreground="green")
                
                self.connect_btn.config(state=tk.DISABLED)
                self.disconnect_btn.config(state=tk.NORMAL)
                self.send_btn.config(state=tk.NORMAL)
                
                self.update_response("Connected to ESP32 successfully!")
                messagebox.showinfo("Success", "Connected to ESP32!")
            else:
                raise Exception("Invalid response")
                
        except Exception as e:
            messagebox.showerror("Connection Error", 
                               f"Could not connect to ESP32 at {ip}\nError: {str(e)}")
            
    def disconnect_esp32(self):
        self.esp32_connected = False
        self.esp32_ip = ""
        self.connection_status.set("Disconnected")
        self.connection_status.config(foreground="red")
        
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.send_btn.config(state=tk.DISABLED)
        
        self.update_response("Disconnected from ESP32")
        
    def send_parameters(self):
        if not self.esp32_connected:
            messagebox.showerror("Error", "Not connected to ESP32")
            return
            
        try:
            pwm = int(float(self.current_pwm.get()))
            delay = int(float(self.current_delay.get()))
            angle = int(float(self.current_angle.get()))
            servo_speed = int(float(self.current_servo_speed.get()))
            
            url = f"http://{self.esp32_ip}/submit?pwm={pwm}&delay={delay}&angle={angle}&servoSpeed={servo_speed}"
            
            response = requests.get(url, timeout=10)
            
            if response.status_code == 200:
                self.update_response(f"Success: {response.text}")
            else:
                self.update_response(f"Error: HTTP {response.status_code}")
                
        except Exception as e:
            self.update_response(f"Error sending command: {str(e)}")
            messagebox.showerror("Error", f"Failed to send command: {str(e)}")
            
    def quick_send(self, pwm=None, delay=None, angle=None, servo_speed=None):
        if not self.esp32_connected:
            messagebox.showerror("Error", "Not connected to ESP32")
            return
            
        # Use current values if not specified
        if pwm is None:
            pwm = int(float(self.current_pwm.get()))
        if delay is None:
            delay = int(float(self.current_delay.get()))
        if angle is None:
            angle = int(float(self.current_angle.get()))
        if servo_speed is None:
            servo_speed = int(float(self.current_servo_speed.get()))
            
        try:
            url = f"http://{self.esp32_ip}/submit?pwm={pwm}&delay={delay}&angle={angle}&servoSpeed={servo_speed}"
            response = requests.get(url, timeout=10)
            
            if response.status_code == 200:
                self.update_response(f"Quick command sent: {response.text}")
                # Update sliders if values were changed
                if pwm != int(float(self.current_pwm.get())):
                    self.current_pwm.set(str(pwm))
                if angle != int(float(self.current_angle.get())):
                    self.current_angle.set(str(angle))
            else:
                self.update_response(f"Error: HTTP {response.status_code}")
                
        except Exception as e:
            self.update_response(f"Error: {str(e)}")
            
    def update_response(self, text):
        current_time = time.strftime("%H:%M:%S")
        message = f"[{current_time}] {text}\n"
        
        self.response_text_widget.insert(tk.END, message)
        self.response_text_widget.see(tk.END)  # Scroll to bottom
        
    def on_closing(self):
        self.stop_camera()
        self.root.destroy()

if __name__ == "__main__":
    root  = tk.Tk()
    app = ESP32ControlGUI(root)
    
    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    root.mainloop()