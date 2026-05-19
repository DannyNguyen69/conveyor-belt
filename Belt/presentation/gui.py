import tkinter as tk
from PIL import Image, ImageTk
from presentation.arena_selector import AreaSelector
class GUI:
# =============================================================================================================== #
#                                                  GUI SETUP                                                      #
# =============================================================================================================== #             
    def __init__(self, root, controller, usecase, state):
        self.root = root
        self.controller = controller
        self.state = state
        self.usecase = usecase
        self.setup_window()
        self.setup_gui()
        self.render_all()
        self.render_area()
    # =========================================================
    # WINDOW
    # =========================================================

    def setup_window(self):

        self.root.title("YOLO Conveyor System")

        self.root.geometry("1200x700")

    # =========================================================
    # GUI
    # =========================================================

    def setup_gui(self):

        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # -------------------------------------------------
        # LEFT PANEL
        # -------------------------------------------------

        left_panel = tk.Frame(main_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        #=====================================================================================================
        # Camera Canvas =====================================================
        #=====================================================================================================
        self.camera_canvas = tk.Canvas(
            left_panel,
            width=800,
            height=500,
            bg="black"
        )

        self.camera_canvas.pack(padx=10, pady=10)
        self.canvas_image = self.camera_canvas.create_image(0, 0, anchor=tk.NW)

        camera_controls = tk.Frame(left_panel)
        camera_controls.pack(fill=tk.X, padx=10)

        self.start_camera_btn = tk.Button(
            camera_controls,
            text="Start Camera",
            command=self.controller.camera.on_start_clicked,
            bg="#4CAF50",
            fg="white",
            padx=10,
            pady=5
        )

        self.start_camera_btn.pack(side=tk.LEFT, padx=5)

        self.stop_camera_btn = tk.Button(
            camera_controls,
            text="Stop Camera",
            command=self.controller.camera.on_stop_clicked,
            bg="#f44336",
            fg="white",
            padx=10,
            pady=5
        )

        self.stop_camera_btn.pack(side=tk.LEFT, padx=5)
        #=====================================================================================================
        # Select Area =====================================================
        #=====================================================================================================
        self.arena_selector = AreaSelector(self.camera_canvas, self.state)
        self.select_btn = tk.Button(
            self.root,
            text="Set Area",
            command= self.arena_selector.start,
            bg="#36f488",
            fg="white",
            padx=10,
            pady=5
        )
        self.select_btn.pack(side=tk.LEFT, padx=5)

        self.clear_btn = tk.Button(
            camera_controls,
            text="Clear Area",
            command=self.arena_selector.clear,
            bg="#f44336",
            fg="white",
            padx=10,
            pady=5
        )

        self.clear_btn.pack(side=tk.LEFT, padx=5)
        #=====================================================================================================
        # Conveyor Controls =====================================================
        #=====================================================================================================

        conveyor_controls = tk.Frame(left_panel)
        conveyor_controls.pack(fill=tk.X, padx=10, pady=10)

        self.start_conveyor_btn = tk.Button(
            conveyor_controls,
            text="START",
            command=lambda: self.controller.conveyor.on_start_clicked(),
            bg="#4CAF50",
            fg="white",
            padx=20,
            pady=8
        )

        self.start_conveyor_btn.pack(side=tk.LEFT, padx=5)

        self.stop_conveyor_btn = tk.Button(
            conveyor_controls,
            text="STOP",
            command=lambda: self.controller.conveyor.on_stop_clicked(),
            bg="#f44336",
            fg="white",
            padx=20,
            pady=8
        )

        self.stop_conveyor_btn.pack(side=tk.LEFT, padx=5)

        # # -------------------------------------------------
        # # RIGHT PANEL
        # # -------------------------------------------------

        right_panel = tk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y)

        # Connection Section

        connection_frame = tk.LabelFrame(
            right_panel,
            text="ESP32 Connection"
        )

        connection_frame.pack(fill=tk.X, padx=10, pady=10)

        self.ip_entry = tk.Entry(connection_frame)

        self.ip_entry.insert(0, "192.168.1.100")

        self.ip_entry.pack(fill=tk.X, padx=5, pady=5)

        self.connect_btn = tk.Button(
            connection_frame,
            text="Connect",
            command=lambda: self.controller.conveyor.on_connect_clicked(self.ip_entry.get()),
            bg="#2196F3",
            fg="white"
        )

        self.connect_btn.pack(side=tk.LEFT, padx=5, pady=5)

        self.disconnect_btn = tk.Button(
            connection_frame,
            text="Disconnect",
            command=lambda: self.controller.conveyor.on_disconnect_clicked(),
            bg="#FF9800",
            fg="white"
        )

        self.disconnect_btn.pack(side=tk.LEFT, padx=5, pady=5)

        # Status

        self.connection_status_label = tk.Label(
            right_panel,
            text=""
        )

        self.connection_status_label.pack(
            anchor=tk.W,
            padx=10,
            pady=5
        )

        self.camera_status_label = tk.Label(
            right_panel,
            text="Camera Stopped",
            bg="#4CAF50",
            foreground="green" 
        )

        self.camera_status_label.pack(
            anchor=tk.W,
            padx=10,
            pady=5
        )

        self.conveyor_status_label = tk.Label(
            right_panel,
            text=""
        )

        self.conveyor_status_label.pack(
            anchor=tk.W,
            padx=10,
            pady=5
        )
        self.setting_area_label = tk.Label(
            right_panel,
            text=""
        )

        self.setting_area_label.pack(
            anchor=tk.W,
            padx=10,
            pady=5
        )
    # =========================================================
    # RENDER
    # =========================================================

    def render_all(self):

        self.render_camera_state()
        self.render_connection_state()
        self.render_conveyor_state()
        self.render_area()

    # ---------------------------------------------------------

    def render_camera_state(self):
        if self.state.camera_running:
            self.start_camera_btn.config(
                state=tk.DISABLED
            )

            self.stop_camera_btn.config(
                state=tk.NORMAL
            )

            self.camera_status_label.config(
                text="Camera: Running", 
                foreground="green"
            )

        else:

            self.start_camera_btn.config(
                state=tk.NORMAL
            )

            self.stop_camera_btn.config(
                state=tk.DISABLED
            )

            self.camera_status_label.config(
                text="Camera: Stopped"
            )

    # ---------------------------------------------------------

    def render_connection_state(self):

        if self.state.esp32_connected:

            self.connect_btn.config(
                state=tk.DISABLED
            )

            self.disconnect_btn.config(
                state=tk.NORMAL
            )

            self.connection_status_label.config(
                text="ESP32: Connected",
                foreground="green"
            )

        else:

            self.connect_btn.config(
                state=tk.NORMAL
            )

            self.disconnect_btn.config(
                state=tk.DISABLED
            )

            self.connection_status_label.config(
                text="ESP32: Disconnected"
            )

    # ---------------------------------------------------------

    def render_conveyor_state(self):

        if self.state.conveyor_running:

            self.start_conveyor_btn.config(
                state=tk.DISABLED
            )

            self.stop_conveyor_btn.config(
                state=tk.NORMAL
            )

            self.conveyor_status_label.config(
                text="Conveyor: Running"
            )

        else:

            self.start_conveyor_btn.config(
                state=tk.NORMAL
            )

            self.stop_conveyor_btn.config(
                state=tk.DISABLED
            )

            self.conveyor_status_label.config(
                text="Conveyor: Stopped"
            )
    def start_ui_loop(self):

        self.update_frame()
        self.update_ui()
        self.root.after(
            30,
            self.start_ui_loop
        )
    def update_ui(self):

        self.render_camera_state()
        self.render_connection_state()
        self.render_conveyor_state()
        self.render_area()

        self.render_frame()
    def update_frame(self):

        frame = self.state.current_frame

        polygon = self.arena_selector.polygon

        self.usecase.process(frame=frame, polygon=polygon)
    def render_frame(self):

        with self.state.frame_lock:

            frame = self.state.current_frame

        if frame is None:
            return

        image = Image.fromarray(frame)

        photo = ImageTk.PhotoImage(image)

        self.camera_canvas.itemconfig(
            self.canvas_image,
            image=photo
        )

        self.camera_canvas.image = photo
    def render_area(self):
        if self.state.beset:
            self.select_btn.config(
                state=tk.DISABLED
            )

            self.clear_btn.config(
                state=tk.NORMAL
            )

            self.setting_area_label.config(
                text="Bi sex"
            )
        else:
            self.select_btn.config(
                state=tk.NORMAL
            )

            self.clear_btn.config(
                state=tk.DISABLED
            )

            self.setting_area_label.config(
                text="Don ki bi sex"
            )