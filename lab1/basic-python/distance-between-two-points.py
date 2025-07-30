import math
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance_to(self, other_point):
        dx = other_point.x - self.x
        dy = other_point.y - self.y
        return math.sqrt(dx**2 + dy**2)


x1 = float(input("enter the value for x1: "))
y1 = float(input("enter the value for y1: "))

x2 = float(input("enter the value for x2: "))
y2 = float(input("enter the value for y2: "))

point1 = Point(x1, y1)
point2 = Point(x2, y2)
distance = point1.distance_to(point2)
print(f"the distance between the given two points is: {distance}")
