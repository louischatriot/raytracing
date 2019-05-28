from math import cos, sin, pi, sqrt, floor


class Vector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def norm_squared(self):
        return self.x * self.x + self.y * self.y + self.z * self.z

    def norm(self):
        return sqrt(self.norm())

    def __str__(self):
        return f"<Vector> {self.x} ; {self.y} ; {self.z}"

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y, self.z - other.z)

    # vector times scalar
    def __mul__(self, other):
        return Vector(self.x * other, self.y * other, self.z * other)

    # scalar product: **
    def __pow__(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z


# a = alpha = angle on (x, y)
# t = theta = angle on ((x,y), z)
def calc_vectors(a, t):
    v = Vector(cos(a) * cos(t), sin(a) * cos(t), sin(t))
    w = Vector(-v.y / cos(t), v.x / cos(t), 0)
    h = Vector(-sin(t) * cos(a), -sin(t) * sin(a), cos(t))

    return v, w, h



# Returns closest intersection point from eye direction to sphere if any
# e_p (vector) = point E, the eye
# m_p (vector) = point M, the pixel on the screen
# c_p (vector) = point C = center of the sphere
# r = sphere radius
def intersect_sphere(e_p, m_p, c_p, r):
    m = m_p - e_p
    ec = c_p - e_p

    # a = m.x * m.x + m.y * m.y + m.z * m.z
    # b = 2 * (m.x * ec.x + m.y * ec.y + m.z * ec.z)
    # c = ec.x ** 2 + ec.y ** 2 + ec.z ** 2 - r ** 2

    a = m.norm_squared()
    b = - 2 * (m ** ec)
    c = ec.norm_squared() - r ** 2

    d = b * b - 4 * a * c
    if d < 0:
        return None

    if d == 0:
        t = -b / (2 * a)
        return e + m * t

    t1 = (-b + sqrt(d)) / (2 * a)
    t2 = (-b - sqrt(d)) / (2 * a)

    i1 = e + m * t1
    i2 = e + m * t2

    if (i1 - e_p).norm() > (i2 - e_p).norm():
        return i2
    else:
        return i1


class Player:
    def __init__(self, a, t, pos = None):
        self.a = a
        self.t = t
        self.pos = pos or Vector(0, 0, 0)


class Scene:
    screen_W = 800
    screen_H = 600

    # Virtual display in front of observer eyes
    # Meters
    display_d = 1
    display_W = 1
    display_H = 0.75

    # Given a player at a position and looking in a direction
    # and a pixel on the screen, return a Vector representing
    # this pixel on the virtual display
    def get_point(player, screen_x, screen_y):
        v, w, h = calc_vectors(player.a, player.t)
        res = player.pos
        res += v * Scene.display_d
        res += w * (Scene.display_W * (screen_x / Scene.screen_W - 0.5))
        res += h * (Scene.display_H * (screen_y / Scene.screen_H - 0.5))
        return res




player = Player(0, 0)

v = Vector(3, 6, 77)
print(v)
print(v * 2)

player.a = pi / 4

print(Scene.get_point(player, 400, 300))



