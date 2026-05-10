W_PRODUCTS = ['W-Cpolygon','W-Circle','W-S-Circle','W-Trapezoid','W-Square']
R_PRODUCTS = ['R-Cpolygon','R-Circle','R-S-Circle','R-Trapezoid','R-Square']
SQUARE_PRODUCTS = ['W-Square', 'R-Square']
class Product:
    def __init__(self, label, bbox, confidence):
        self.label = label
        self.bbox = bbox
        self.confidence = confidence
    def center(self):
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) / 2, (y1 + y2) / 2)
    @property
    def is_square(self):
        return self.label in SQUARE_PRODUCTS
    @property
    def is_wrong(self):
        return self.label in W_PRODUCTS
    @property
    def is_right(self):
        return self.label in R_PRODUCTS