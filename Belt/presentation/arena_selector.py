
class AreaSelector:

    def __init__(self, canvas, state):

        self.canvas = canvas

        self.points = []

        self.state = state

        self.canvas.bind("<Button-1>", self.on_click)

    # ==========================================
    # START
    # ==========================================

    def start(self):

        self.clear()

        self.state.is_setting = True

        print("Click 4 points")

    # ==========================================
    # CLICK EVENT
    # ==========================================

    def on_click(self, event):

        if not self.state.is_setting:
            return

        if len(self.points) >= 4:
            return

        x = event.x
        y = event.y

        self.points.append((x, y))

        self.draw_point(x, y, len(self.points))

        # draw line
        if len(self.points) > 1:

            x1, y1 = self.points[-2]
            x2, y2 = self.points[-1]

            self.canvas.create_line(
                x1, y1,
                x2, y2,
                fill="green",
                width=2,
                tags="polygon"
            )

        # close polygon
        if len(self.points) == 4:

            x1, y1 = self.points[-1]
            x2, y2 = self.points[0]

            self.canvas.create_line(
                x1, y1,
                x2, y2,
                fill="green",
                width=2,
                tags="polygon"
            )

            self.state.is_setting = False
            self.state.beset = True
            print("Polygon completed")
            

    # ==========================================
    # DRAW POINT
    # ==========================================

    def draw_point(self, x, y, index):

        r = 5

        self.canvas.create_oval(
            x-r, y-r,
            x+r, y+r,
            fill="red",
            outline="white",
            width=2,
            tags="point"
        )

        self.canvas.create_text(
            x+10,
            y-10,
            text=str(index),
            fill="yellow",
            font=("Arial", 10, "bold"),
            tags="point"
        )

    # ==========================================
    # CLEAR
    # ==========================================

    def clear(self):

        self.points = []

        self.state.is_setting = False

        self.canvas.delete("point")
        self.canvas.delete("polygon")
        self.state.beset = False

    # ==========================================
    # GET POLYGON
    # ==========================================

    @property
    def polygon(self):

        return self.points

    # ==========================================
    # CHECK POINT INSIDE
    # ==========================================

    def contains(self, point):

        if len(self.points) < 4:
            return False

        x, y = point

        polygon = self.points

        n = len(polygon)

        inside = False

        p1x, p1y = polygon[0]

        for i in range(1, n + 1):

            p2x, p2y = polygon[i % n]

            if y > min(p1y, p2y):

                if y <= max(p1y, p2y):

                    if x <= max(p1x, p2x):

                        if p1y != p2y:

                            xinters = (
                                (y - p1y)
                                * (p2x - p1x)
                                / (p2y - p1y)
                                + p1x
                            )

                        if p1x == p2x or x <= xinters:

                            inside = not inside

            p1x, p1y = p2x, p2y

        return inside