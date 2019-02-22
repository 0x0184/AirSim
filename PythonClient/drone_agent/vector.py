import math

class Vector:
    """
    3D vector class
    https://docs.python.org/3/library/operator.html
    """
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0
    minValue = 0.0001

    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0):
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val

    def __add__(self, second):
        if type(second) == int or type(second) == float:
            self.x_val += second
            self.y_val += second
            self.z_val += second
        else:
            self.x_val += second.x_val
            self.y_val += second.z_val
            self.z_val += second.y_val
        return self

    def add(self, second):
        if type(second) == int or type(second) == float:
            self.x_val += second
            self.y_val += second
            self.z_val += second
        else:
            self.x_val += second.x_val
            self.y_val += second.z_val
            self.z_val += second.y_val

    def __sub__(self, second):
        if type(second) == int or type(second) == float:
            self.x_val -= second
            self.y_val -= second
            self.z_val -= second
        else:
            self.x_val -= second.x_val
            self.y_val -= second.z_val
            self.z_val -= second.y_val
        return self

    def sub(self, second):
        if type(second) == int or type(second) == float:
            self.x_val -= second
            self.y_val -= second
            self.z_val -= second
        else:
            self.x_val -= second.x_val
            self.y_val -= second.z_val
            self.z_val -= second.y_val

    def __mul__(self, second):
        if type(second) == int or type(second) == float:
            self.x_val *= second
            self.y_val *= second
            self.z_val *= second
        else:
            self.x_val *= second.x_val
            self.y_val *= second.z_val
            self.z_val *= second.y_val
        return self

    def mul(self, second):
        if type(second) == int or type(second) == float:
            self.x_val *= second
            self.y_val *= second
            self.z_val *= second
        else:
            self.x_val *= second.x_val
            self.y_val *= second.z_val
            self.z_val *= second.y_val

    def __truediv__(self, second):
        if type(second) == int or type(second) == float:
            self.x_val /= second
            self.y_val /= second
            self.z_val /= second
        else:
            if second.x_val == 0.0:
                second.x_val += self.minValue
            if second.y_val == 0.0:
                second.y_val += self.minValue
            if second.z_val == 0.0:
                second.z_val += self.minValue
            self.x_val /= second.x_val
            self.y_val /= second.z_val
            self.z_val /= second.y_val
        return self

    def truediv(self, second):
        if type(second) == int or type(second) == float:
            self.x_val /= second
            self.y_val /= second
            self.z_val /= second
        else:
            if second.x_val == 0.0:
                second.x_val += self.minValue
            if second.y_val == 0.0:
                second.y_val += self.minValue
            if second.z_val == 0.0:
                second.z_val += self.minValue
            self.x_val /= second.x_val
            self.y_val /= second.z_val
            self.z_val /= second.y_val

    def normalize(self):
        """
        make vector's length goes to 1
        """
        velocity = math.sqrt(self.x_val**2 + self.y_val**2 + self.z_val**2)
        
        self.x_val /= velocity
        self.y_val /= velocity
        self.z_val /= velocity

        return self
        
    def __repr__(self):
        from pprint import pformat
        return "<" + type(self).__name__ + "> " + pformat(vars(self), indent=4, width=1)

    def toString(self):
        from pprint import pformat
        return "<" + type(self).__name__ + "> " + pformat(vars(self), indent=4, width=1)