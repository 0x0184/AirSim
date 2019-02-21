import haversine
import math
from vector import Vector

class LocalMap:
    """
    Map for GPS map to 2D map
    The map is stretched from center latitude and longitude
    The map is optimized for Korea area so the map is not supported border of longitude 180 and -180
    """
    def __init__(self, coords=[], UE=False):
        """
        Initialize map by GPS data
        coords : [(lat, lon), (lat, lon), ...]
        """
        self._UE = UE
        if self._UE:
            pass
        else:
            if coords is []:
                """
                The map is initialized for Korea
                North : 43˚00'42"(43.011667)    (lat)
                East : 131˚52'22"(131.872778)   (lon)
                West : 124˚10'51"(124.180833)   (lon)
                South : 33˚06'43"(33.111944)    (lat)
                Center : 38˚00'00"(38)          (lat)
                Center : 127˚30'00"(127.5)      (lon)
                """
                self._min_lat = 33.111944
                self._max_lat = 43.011667
                self._min_lon = 124.180833
                self._max_lon = 131.872778
                self._center_lat = 38
                self._center_lon = 127.5
            else:
                # Initialize for specific area
                lats = []
                lons = []
                for i in range(len(coords)):
                    lats.append(coords[i][0])
                    lons.append(coords[i][1])
                self._min_lat = min(lats)
                self._max_lat = max(lats)
                self._min_lon = min(lons)
                self._max_lon = max(lons)
                self._center_lat = (self._min_lat + self._max_lat) / 2
                self._center_lon = (self._min_lon + self._max_lon) / 2

    def location2D(self, coord):
        """
        coord : tuple likes (lat, lon)
        return : (x, y)
        """
        x = haversine.Haversine2D((self._center_lat, self._center_lon), (self._center_lat, coord[1]))
        y = haversine.Haversine2D((self._center_lat, self._center_lon), (coord[0], self._center_lon))

        if coord[1] < self._center_lon:
            x *= -1
        if coord[0] < self._center_lat:
            y *= -1
        return (x, y)

    def location3D(self, coord):
        """
        coord : tuple likes (lat, lon, alt)
        return : (x, y, z)
        """
        x = haversine.Haversine2D((self._center_lat, self._center_lon), (self._center_lat, coord[1]))
        y = haversine.Haversine2D((self._center_lat, self._center_lon), (coord[0], self._center_lon))
        z = coord[2]

        if coord[1] < self._center_lon:
            x *= -1
        if coord[0] < self._center_lat:
            y *= -1
        return (x, y, z)

def minute_to_decimal(degree, minute, second):
    """
    Change degree minute second to decimal degree
    """
    return degree + minute / 60 + second / 3600

def decimal_to_minute(decimal):
    """
    Change decimal degree to (degree, minute, second)
    """
    degree = int(decimal)
    minute = int((decimal - degree) * 60)
    second = int((decimal - degree - minute * 60) * 3600)
    return (degree, minute, second)

def distance2D(loc2d1=tuple(), loc2d2=tuple()):
    """
    get distance from (x1, y1), (x2, y2)
    loc2d1 : (x1, y1)
    loc2d2 : (x2, y2)
    return : distance
    """
    return math.sqrt((loc2d1[0] - loc2d2[0])**2 + (loc2d1[1] - loc2d2[1])**2)

def distance3D(loc3d1=tuple(), loc3d2=tuple()):
    """
    get distance from (x1, y1, z1), (x2, y2, z2)
    loc3d1 : (x1, y1, z1)
    loc3d2 : (x2, y2, z2)
    return : distance
    """
    return math.sqrt((loc3d1[0] - loc3d2[0])**2 + (loc3d1[1] - loc3d2[1])**2 + (loc3d1[2] - loc3d2[2])**2)

def distance2Dv(loc2d1=Vector(), loc2d2=Vector()):
    """
    get distance from (x_val1, y_val1), (x_val2, y_val2)
    loc2d1 : (x_val1, y_val1)
    loc2d2 : (x_val2, y_val2)
    return : distance
    """
    return math.sqrt((loc2d1.x_val - loc2d2.x_val)**2 + (loc2d1.y_val - loc2d2.y_val)**2)

def distance3Dv(loc3d1=Vector(), loc3d2=Vector()):
    """
    get distance from (x_val1, y_val1, z_val1), (x_val2, y_val2, z_val2)
    loc3d1 : (x_val1, y_val1, z_val1)
    loc3d2 : (x_val2, y_val2, z_val2)
    return : distance
    """
    return math.sqrt((loc3d1.x_val - loc3d2.x_val)**2 + (loc3d1.y_val - loc3d2.y_val)**2 + (loc3d1.z_val - loc3d2.z_val)**2)