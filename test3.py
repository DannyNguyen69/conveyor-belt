import cv2
import numpy as np
import pygame
import sys
import snap7
from snap7.util import *
try:
    from scapy.all import ARP, Ether, srp
    SCAPY_AVAILABLE = True
except ImportError:
    print("Warning: scapy not available, PLC network scanning will be disabled")
    SCAPY_AVAILABLE = False
import threading
import time
from ultralytics import YOLO
import torch

# Khởi tạo Pygame
pygame.init()

# Kiểm tra và cấu hình CUDA
if torch.cuda.is_available():
    device = torch.device('cuda')
    print(f"CUDA available: {torch.cuda.get_device_name(0)}")
    print(f"GPU Memory: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.1f} GB")
    
    # Kiểm tra tất cả GPU có sẵn
    print(f"Tổng số GPU: {torch.cuda.device_count()}")
    for i in range(torch.cuda.device_count()):
        gpu_name = torch.cuda.get_device_name(i)
        gpu_memory = torch.cuda.get_device_properties(i).total_memory / 1024**3
        print(f"GPU {i}: {gpu_name} - {gpu_memory:.1f}GB")
        
        # Ưu tiên RTX 3050 nếu có
        if "RTX 3050" in gpu_name or "3050" in gpu_name:
            device = torch.device(f'cuda:{i}')
            print(f"Chọn GPU: {gpu_name} (GPU {i})")
            break
else:
    device = torch.device('cpu')
    print("CUDA not available, using CPU")

# Kích thước cửa sổ và camera
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 360

# Màu sắc
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (128, 128, 128)
BLUE = (0, 100, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
LIGHT_BLUE = (100, 200, 255)

# Kích thước và vị trí khung PLC
PLC_FRAME_WIDTH = 300
PLC_FRAME_HEIGHT = 200  # Tăng chiều cao để hiển thị danh sách IP
PLC_FRAME_X = WINDOW_WIDTH - PLC_FRAME_WIDTH - 20
PLC_FRAME_Y = 20

# Font chữ
font = pygame.font.Font(None, 36)
small_font = pygame.font.Font(None, 24)

class Camera:
    def __init__(self, camera_id=0):
        self.camera_id = camera_id
        self.cap = None
        self.is_active = False
        self.frame = None
        self.yolo_model = None
        self.detection_results = []
        self.fps = 0
        self.frame_count = 0
        self.last_time = time.time()
        
    def start(self):
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            # Thêm timeout cho camera
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            if self.cap.isOpened():
                self.is_active = True
                # Tải model YOLO
                try:
                    print(f"Đang tải model YOLO cho camera {self.camera_id}...")
                    self.yolo_model = YOLO('yolov8n_custom.pt')
                    # Chuyển model lên GPU nếu có CUDA
                    if torch.cuda.is_available():
                        self.yolo_model.to(device)
                        print(f"Model camera {self.camera_id} loaded on GPU: {torch.cuda.get_device_name(device.index if hasattr(device, 'index') else 0)}")
                    else:
                        print(f"Model camera {self.camera_id} loaded on CPU")
                except Exception as e:
                    print(f"Không thể tải model YOLO cho camera {self.camera_id}: {e}")
                    print("Sẽ chạy camera không có detection")
                return True
            else:
                print(f"Không thể mở camera {self.camera_id}")
                return False
        except Exception as e:
            print(f"Lỗi khởi tạo camera {self.camera_id}: {e}")
            return False
    
    def stop(self):
        self.is_active = False
        if self.cap:
            self.cap.release()
    
    def get_frame(self):
        if not self.is_active or not self.cap:
            return None
        
        try:
            # Thêm timeout cho việc đọc frame
            ret, frame = self.cap.read()
            if ret and frame is not None:
                # Tính FPS
                self.frame_count += 1
                current_time = time.time()
                if current_time - self.last_time >= 1.0:
                    self.fps = self.frame_count / (current_time - self.last_time)
                    self.frame_count = 0
                    self.last_time = current_time
                
                # Resize frame
                frame = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT))
                
                # Chạy YOLO detection nếu có model
                if self.yolo_model:
                    try:
                        # Sử dụng CUDA nếu có sẵn
                        if torch.cuda.is_available():
                            results = self.yolo_model(frame, device=device)
                        else:
                            results = self.yolo_model(frame)
                        
                        # Chuyển kết quả về CPU để xử lý
                        if torch.cuda.is_available():
                            self.detection_results = results[0].boxes.data.cpu().numpy()
                        else:
                            self.detection_results = results[0].boxes.data.numpy()
                        
                        # Vẽ bounding boxes
                        for detection in self.detection_results:
                            x1, y1, x2, y2, conf, cls = detection
                            if conf > 0.5:  # Chỉ hiển thị detection có confidence > 50%
                                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                                cv2.putText(frame, f'{self.yolo_model.names[int(cls)]} {conf:.2f}', 
                                          (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    except Exception as e:
                        print(f"Lỗi YOLO detection camera {self.camera_id}: {e}")
                        self.detection_results = np.array([])
                
                # Chuyển đổi từ BGR sang RGB
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.frame = frame
                return frame
            else:
                # Nếu không đọc được frame, thử khởi động lại camera
                print(f"Camera {self.camera_id} không đọc được frame, thử khởi động lại...")
                self.stop()
                time.sleep(0.1)
                self.start()
                return None
                
        except Exception as e:
            print(f"Lỗi đọc frame camera {self.camera_id}: {e}")
            return None
    
    def get_pygame_surface(self):
        if self.frame is not None:
            # Chuyển đổi numpy array thành pygame surface
            frame_surface = pygame.surfarray.make_surface(self.frame.swapaxes(0, 1))
            return frame_surface
        return None

    def get_ok_pr_percentage(self):
        """Tính phần trăm confidence cao nhất của nhãn Ok_pr"""
        if len(self.detection_results) == 0 or not self.yolo_model:
            return 0.0
        
        max_confidence = 0.0
        
        for detection in self.detection_results:
            x1, y1, x2, y2, conf, cls = detection
            label_name = self.yolo_model.names[int(cls)]
            if label_name == "Ok_pr":
                # Lấy confidence cao nhất của nhãn Ok_pr
                if conf > max_confidence:
                    max_confidence = conf
        
        # Trả về % confidence (0-100)
        return max_confidence * 100

class PLCConnection:
    def __init__(self):
        self.ip = ""
        self.is_connected = False
        self.is_typing = False
        self.is_scanning = False
        self.scan_results = []
        self.selected_ip = None
        self.rect = pygame.Rect(PLC_FRAME_X + 10, PLC_FRAME_Y + 40, 200, 30)
        self.connect_button = pygame.Rect(PLC_FRAME_X + 220, PLC_FRAME_Y + 40, 70, 30)
        self.plc = snap7.client.Client()
        self.scan_thread = None
        self.connection_status = "Disconnected"

    def scan_network(self):
        self.is_scanning = True
        self.scan_results = []
        
        if not SCAPY_AVAILABLE:
            print("Scapy not available, skipping network scan")
            self.is_scanning = False
            return
        
        try:
            # Tạo ARP request packet
            arp = ARP(pdst="192.168.0.0/24")  # Thay đổi subnet mask phù hợp với mạng của bạn
            ether = Ether(dst="ff:ff:ff:ff:ff:ff")
            packet = ether/arp

            # Gửi packet và nhận response
            result = srp(packet, timeout=3, verbose=0)[0]
            
            # Lọc các thiết bị có thể là PLC (dựa trên MAC address)
            for sent, received in result:
                ip = received.psrc
                mac = received.hwsrc
                # Thêm vào danh sách nếu MAC address có thể là của PLC
                if mac.startswith(("00:1b:1b", "00:1b:1c", "00:1b:1d")):  # MAC prefixes của Siemens PLC
                    self.scan_results.append(ip)
        except Exception as e:
            print(f"Scan error: {e}")
        
        self.is_scanning = False

    def try_connect(self):
        if not self.selected_ip:
            return False
        
        try:
            # Kết nối với PLC
            self.plc.connect(self.selected_ip, 0, 1)  # rack=0, slot=1
            if self.plc.get_connected():
                self.is_connected = True
                self.connection_status = "Connected"
                return True
        except Exception as e:
            print(f"Connection error: {e}")
            self.connection_status = f"Error: {str(e)}"
        
        return False

    def draw(self, screen):
        # Vẽ khung PLC
        pygame.draw.rect(screen, GRAY, (PLC_FRAME_X, PLC_FRAME_Y, PLC_FRAME_WIDTH, PLC_FRAME_HEIGHT), 2)
        
        # Vẽ tiêu đề
        title = font.render("PLC Connection", True, WHITE)
        screen.blit(title, (PLC_FRAME_X + 10, PLC_FRAME_Y + 5))
        
        # Vẽ ô nhập IP
        pygame.draw.rect(screen, WHITE, self.rect, 2)
        ip_text = small_font.render(self.ip if self.ip else "Enter IP...", True, WHITE)
        screen.blit(ip_text, (self.rect.x + 5, self.rect.y + 5))
        
        # Vẽ nút kết nối
        pygame.draw.rect(screen, BLUE, self.connect_button)
        connect_text = small_font.render("Connect", True, WHITE)
        screen.blit(connect_text, (self.connect_button.x + 5, self.connect_button.y + 5))
        
        # Vẽ trạng thái kết nối
        status_color = GREEN if self.is_connected else RED
        status = small_font.render(self.connection_status, True, status_color)
        screen.blit(status, (PLC_FRAME_X + 10, PLC_FRAME_Y + 80))

        # Vẽ danh sách IP đã quét
        if self.is_scanning:
            scan_text = small_font.render("Scanning...", True, LIGHT_BLUE)
            screen.blit(scan_text, (PLC_FRAME_X + 10, PLC_FRAME_Y + 110))
        else:
            for i, ip in enumerate(self.scan_results):
                y_pos = PLC_FRAME_Y + 110 + (i * 25)
                if y_pos < PLC_FRAME_Y + PLC_FRAME_HEIGHT - 10:
                    color = LIGHT_BLUE if ip == self.selected_ip else WHITE
                    ip_text = small_font.render(ip, True, color)
                    screen.blit(ip_text, (PLC_FRAME_X + 10, y_pos))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.is_typing = True
                # Bắt đầu quét khi click vào ô nhập IP
                if not self.is_scanning and not self.scan_thread:
                    self.scan_thread = threading.Thread(target=self.scan_network)
                    self.scan_thread.start()
            elif self.connect_button.collidepoint(event.pos):
                if self.try_connect():
                    self.connection_status = "Connected"
                else:
                    self.connection_status = "Connection failed"
            else:
                self.is_typing = False
                # Kiểm tra click vào IP trong danh sách
                mouse_y = event.pos[1]
                for i, ip in enumerate(self.scan_results):
                    y_pos = PLC_FRAME_Y + 110 + (i * 25)
                    if y_pos <= mouse_y <= y_pos + 20:
                        self.selected_ip = ip
                        self.ip = ip
                        break
        
        if event.type == pygame.KEYDOWN and self.is_typing:
            if event.key == pygame.K_RETURN:
                self.is_typing = False
            elif event.key == pygame.K_BACKSPACE:
                self.ip = self.ip[:-1]
            else:
                # Chỉ cho phép nhập số và dấu chấm
                if event.unicode.isdigit() or event.unicode == '.':
                    self.ip += event.unicode

def main():
    # Tạo cửa sổ Pygame
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("YOLO Dual Camera System with PLC")
    
    # Khởi tạo PLC connection
    plc = PLCConnection()
    
    # Khởi tạo camera
    camera1 = Camera(0)
    camera2 = Camera(1)
    
    # Khởi động camera
    print("Đang khởi động camera 1...")
    camera1_started = camera1.start()
    print("Đang khởi động camera 2...")
    camera2_started = camera2.start()
    
    # Thử camera khác nếu không khả dụng
    if not camera1_started:
        print("⚠️ Camera 1 không khả dụng, thử camera khác...")
        for cam_id in [2, 3, 4]:
            print(f"Thử camera {cam_id}...")
            camera1 = Camera(cam_id)
            camera1_started = camera1.start()
            if camera1_started:
                break
    
    if not camera2_started:
        print("⚠️ Camera 2 không khả dụng, thử camera khác...")
        for cam_id in [2, 3, 4]:
            if cam_id != camera1.camera_id:  # Tránh trùng camera
                print(f"Thử camera {cam_id}...")
                camera2 = Camera(cam_id)
                camera2_started = camera2.start()
                if camera2_started:
                    break
    
    print(f"Camera 1 (ID {camera1.camera_id}): {'✅ Hoạt động' if camera1_started else '❌ Không khả dụng'}")
    print(f"Camera 2 (ID {camera2.camera_id}): {'✅ Hoạt động' if camera2_started else '❌ Không khả dụng'}")
    
    # Nếu không có camera nào, tạo camera giả để test
    if not camera1_started and not camera2_started:
        print("⚠️ Không có camera nào khả dụng, tạo camera test...")
        # Tạo frame test màu đen
        test_frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
        camera1.frame = test_frame
        camera1.is_active = True
        camera2.frame = test_frame
        camera2.is_active = True
    
    # Font chữ
    font = pygame.font.Font(None, 36)
    
    running = True
    while running:
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                
                # Xử lý sự kiện cho PLC connection
                plc.handle_event(event)
            
            # Lấy frame từ camera
            try:
                camera1.get_frame()
                camera2.get_frame()
            except Exception as e:
                print(f"Lỗi khi lấy frame: {e}")
            
            # Xóa màn hình
            screen.fill(BLACK)
            
            # Vẽ đường phân cách
            pygame.draw.line(screen, GRAY, (CAMERA_WIDTH, 0), (CAMERA_WIDTH, WINDOW_HEIGHT), 2)
            
            # Vẽ tiêu đề cho camera
            try:
                camera1_percentage = camera1.get_ok_pr_percentage()
                camera2_percentage = camera2.get_ok_pr_percentage()
            except Exception as e:
                print(f"Lỗi khi tính %: {e}")
                camera1_percentage = 0.0
                camera2_percentage = 0.0
            
            # Vẽ heading cho Camera 1
            text1 = font.render("Camera 1", True, WHITE)
            percentage_text1 = small_font.render(f"Ok_pr: {camera1_percentage:.1f}%", True, GREEN)
            screen.blit(text1, (10, 10))
            screen.blit(percentage_text1, (180, 15))

            # Vẽ khung camera 1
            if camera1.is_active:
                camera1_surface = camera1.get_pygame_surface()
                if camera1_surface:
                    screen.blit(camera1_surface, (0, 40))
                else:
                    error_text = small_font.render("Camera 1 không khả dụng", True, RED)
                    screen.blit(error_text, (10, 50))
            else:
                error_text = small_font.render("Camera 1 không được kết nối", True, RED)
                screen.blit(error_text, (10, 50))

            # Vẽ heading cho Camera 2 (ngay trên khung chia)
            text2 = font.render("Camera 2", True, WHITE)
            percentage_text2 = small_font.render(f"Ok_pr: {camera2_percentage:.1f}%", True, GREEN)
            screen.blit(text2, (10, CAMERA_HEIGHT + 40))
            screen.blit(percentage_text2, (180, CAMERA_HEIGHT + 45))

            # Vẽ khung camera 2
            if camera2.is_active:
                camera2_surface = camera2.get_pygame_surface()
                if camera2_surface:
                    screen.blit(camera2_surface, (0, CAMERA_HEIGHT + 70))
                else:
                    error_text = small_font.render("Camera 2 không khả dụng", True, RED)
                    screen.blit(error_text, (10, CAMERA_HEIGHT + 80))
            else:
                error_text = small_font.render("Camera 2 không được kết nối", True, RED)
                screen.blit(error_text, (10, CAMERA_HEIGHT + 80))
            
            # Vẽ PLC connection interface
            try:
                plc.draw(screen)
            except Exception as e:
                print(f"Lỗi khi vẽ PLC: {e}")
                
            # Cập nhật màn hình
            pygame.display.flip()
            
            # Giới hạn FPS
            pygame.time.Clock().tick(30)
            
        except Exception as e:
            print(f"Lỗi trong main loop: {e}")
            continue
    
    # Dọn dẹp
    camera1.stop()
    camera2.stop()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main() 