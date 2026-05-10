W_PRODUCTS = ['W-Cpolygon','W-Circle','W-S-Circle','W-Trapezoid','W-Square']
R_PRODUCTS = ['R-Cpolygon','R-Circle','R-S-Circle','R-Trapezoid','R-Square']
SQUARE_PRODUCTS = ['W-Square', 'R-Square']
class Rule():
    def __init__(self, label):
        self.label = label
    def RuleSquare(self):
        return self.label in SQUARE_PRODUCTS
    def RuleWrong(self):
        return self.label in W_PRODUCTS