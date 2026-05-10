class DetectionService:
    def __init__(self, square_products, trigger_products):
        self.square_products = square_products
        self.trigger_products = trigger_products

    def is_inside(self, product, polygon, pip):
        cx, cy = product.center()
        return pip((int(cx), int(cy)), polygon)

    def should_stop(self, product, polygon, pip):
        return (
            product.label in self.square_products and
            self.is_inside(product, polygon, pip)
        )

    def should_trigger(self, product):
        return product.label in self.trigger_products