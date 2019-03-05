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
        vec = self.copy()
        if type(second) == int or type(second) == float:
            vec.x_val += second
            vec.y_val += second
            vec.z_val += second
        else:
            vec.x_val += second.x_val
            vec.y_val += second.y_val
            vec.z_val += second.z_val
        return vec

    def add(self, second):
        if type(second) == int or type(second) == float:
            self.x_val += second
            self.y_val += second
            self.z_val += second
        else:
            self.x_val += second.x_val
            self.y_val += second.y_val
            self.z_val += second.z_val

    def __sub__(self, second):
        vec = self.copy()
        if type(second) == int or type(second) == float:
            vec.x_val -= second
            vec.y_val -= second
            vec.z_val -= second
        else:
            vec.x_val -= second.x_val
            vec.y_val -= second.y_val
            vec.z_val -= second.z_val
        return vec

    def sub(self, second):
        if type(second) == int or type(second) == float:
            self.x_val -= second
            self.y_val -= second
            self.z_val -= second
        else:
            self.x_val -= second.x_val
            self.y_val -= second.y_val
            self.z_val -= second.z_val

    def __mul__(self, second):
        vec = self.copy()
        if type(second) == int or type(second) == float:
            vec.x_val *= second
            vec.y_val *= second
            vec.z_val *= second
        else:
            vec.x_val *= second.x_val
            vec.y_val *= second.y_val
            vec.z_val *= second.z_val
        return vec

    def mul(self, second):
        if type(second) == int or type(second) == float:
            self.x_val *= second
            self.y_val *= second
            self.z_val *= second
        else:
            self.x_val *= second.x_val
            self.y_val *= second.y_val
            self.z_val *= second.z_val

    def __truediv__(self, second):
        vec = self.copy()
        if type(second) == int or type(second) == float:
            vec.x_val /= second
            vec.y_val /= second
            vec.z_val /= second
        else:
            vec.x_val /= second.x_val
            vec.y_val /= second.y_val
            vec.z_val /= second.z_val
        return vec

    def truediv(self, second):
        if type(second) == int or type(second) == float:
            self.x_val /= second
            self.y_val /= second
            self.z_val /= second
        else:
            self.x_val /= second.x_val
            self.y_val /= second.y_val
            self.z_val /= second.z_val

    def size(self):
        return math.sqrt(self.x_val**2 + self.y_val**2 + self.z_val**2)

    def size2D(self):
        return math.sqrt(self.x_val**2 + self.y_val**2)

    def normalize(self):
        """
        make vector's length goes to 1
        """
        velocity = self.size()

        if velocity == 0:
            return Vector()
        
        self.x_val /= velocity
        self.y_val /= velocity
        self.z_val /= velocity

        return self

    def normalize2D(self):
        """
        make vector's length goes to 1
        """
        velocity = self.size2D()

        if velocity == 0:
            return Vector()
        
        self.x_val /= velocity
        self.y_val /= velocity
        self.z_val = 0.0

        return self

    def make_steer(self, max_speed):
        """
        control steer within max speed like limit
        """
        distance = self.size()
        
        if distance > max_speed:
            self.x_val = self.x_val / distance * max_speed
            self.y_val = self.y_val / distance * max_speed
            self.z_val = self.z_val / distance * max_speed
            return self
        else:
            return self

    def turn_left(self):
        steer = self.copy()
        steer.y_val = math.cos(math.pi/2) * self.y_val - math.sin(math.pi/2) * self.x_val
        steer.x_val = math.sin(math.pi/2) * self.y_val + math.cos(math.pi/2) * self.x_val
        return steer

    def turn_right(self):
        steer = self.copy()
        steer.y_val = math.cos(-math.pi/2) * self.y_val - math.sin(-math.pi/2) * self.x_val
        steer.x_val = math.sin(-math.pi/2) * self.y_val + math.cos(-math.pi/2) * self.x_val
        return steer

    def turn_around(self):
        steer = self.copy()
        steer.y_val = math.cos(-math.pi) * self.y_val - math.sin(-math.pi) * self.x_val
        steer.x_val = math.sin(-math.pi) * self.y_val + math.cos(-math.pi) * self.x_val
        return steer

    def copy(self):
        """
        copy vector
        """
        return Vector(x_val=self.x_val, y_val=self.y_val, z_val=self.z_val)
        
    def __repr__(self):
        from pprint import pformat
        return "<" + type(self).__name__ + "> " + pformat(vars(self), indent=4, width=1)

    def toString(self):
        from pprint import pformat
        return "<" + type(self).__name__ + "> " + pformat(vars(self), indent=4, width=1)