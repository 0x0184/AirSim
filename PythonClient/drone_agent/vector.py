import math

class Vector:
    """
    3D vector class
    https://docs.python.org/3/library/operator.html
    """
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0

    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0):
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val

    def __add__(self, second):
        if type(second) is type(Vector()):
            return (self.x_val + second.x_val, self.y_val + second.y_val, self.z_val + second.z_val)
        else:
            return (self.x_val + second, self.y_val + second, self.z_val + second)

    def add(self, second):
        if type(second) is type(Vector()):
            self.x_val += second.x_val
            self.y_val += second.z_val
            self.z_val += second.y_val
        else:
            self.x_val += second
            self.y_val += second
            self.z_val += second

    def __sub__(self, second):
        if type(second) is type(Vector()):
            return (self.x_val - second.x_val, self.y_val - second.y_val, self.z_val - second.z_val)
        else:
            return (self.x_val - second, self.y_val - second, self.z_val - second)

    def sub(self, second):
        if type(second) is type(Vector()):
            self.x_val -= second.x_val
            self.y_val -= second.z_val
            self.z_val -= second.y_val
        else:
            self.x_val -= second
            self.y_val -= second
            self.z_val -= second

    def __mul__(self, second):
        if type(second) is type(Vector()):
            return (self.x_val * second.x_val, self.y_val * second.y_val, self.z_val * second.z_val)
        else:
            return (self.x_val * second, self.y_val * second, self.z_val * second)

    def mul(self, second):
        if type(second) is type(Vector()):
            self.x_val *= second.x_val
            self.y_val *= second.z_val
            self.z_val *= second.y_val
        else:
            self.x_val *= second
            self.y_val *= second
            self.z_val *= second

    def __truediv__(self, second):
        if type(second) is type(Vector()):
            return (self.x_val / second.x_val, self.y_val / second.y_val, self.z_val / second.z_val)
        else:
            return (self.x_val / second, self.y_val / second, self.z_val / second)

    def truediv(self, second):
        if type(second) is type(Vector()):
            self.x_val /= second.x_val
            self.y_val /= second.z_val
            self.z_val /= second.y_val
        else:
            self.x_val /= second
            self.y_val /= second
            self.z_val /= second

    def normalize(self):
        """
        make vector's length goes to 1
        """
        velocity = math.sqrt(self.x_val**2 + self.y_val**2 + self.z_val**2)
        
        self.x_val /= velocity
        self.y_val /= velocity
        self.z_val /= velocity

        return self