from math import cos, sin, pi, sqrt, floor

# sudo apt-get install cairo
# pip install pycairo
import cairo

# sudo apt install libgirepository1.0-dev gcc libcairo2-dev pkg-config python3-dev gir1.2-gtk-3.0
# pip3 install PyGObject
import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk


# For testing
SIZE = 30


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


def draw(da, ctx):
    print("EEEEEEE")
    print(da)
    print(ctx)


class Scene:
    def __init__(self):
        # Screen (a gtk window)
        self.screen_W = 800
        self.screen_H = 600
        self.ctx = None

        win = Gtk.Window()
        win.connect('destroy', lambda w: Gtk.main_quit())
        win.set_default_size(self.screen_W, self.screen_H)

        drawingarea = Gtk.DrawingArea()
        win.add(drawingarea)
        drawingarea.connect('draw', self.draw_frame)

        win.show_all()
        Gtk.main()

        # Virtual display in front of observer eyes
        # Meters
        self.display_d = 1
        self.display_W = 1
        self.display_H = 0.75

    def draw_frame(self, da, ctx):
        if not self.ctx:
            self.ctx = ctx

        ctx.set_source_rgb(0, 0, 0)

        ctx.set_line_width(SIZE / 4)
        ctx.set_tolerance(0.1)


        ctx.move_to(0, 0)
        ctx.rel_line_to(2 * SIZE, 0)
        ctx.rel_line_to(0, 2 * SIZE)
        ctx.rel_line_to(-2 * SIZE, 0)
        ctx.close_path()

    # Given a player at a position and looking in a direction
    # and a pixel on the screen, return a Vector representing
    # this pixel on the virtual display
    def get_point(self, player, screen_x, screen_y):
        v, w, h = calc_vectors(player.a, player.t)
        res = player.pos
        res += v * self.display_d
        res += w * (self.display_W * (screen_x / self.screen_W - 0.5))
        res += h * (self.display_H * (screen_y / self.screen_H - 0.5))
        return res


    def draw_pixel(self, x, y, color):
        ctx = self.ctx

        print("==================")
        print(ctx)

        ctx.set_source_rgb(0, 0, 0)

        ctx.set_line_width(SIZE / 4)
        ctx.set_tolerance(0.1)


        ctx.move_to(0, 0)
        ctx.rel_line_to(2 * SIZE, 0)
        ctx.rel_line_to(0, 2 * SIZE)
        ctx.rel_line_to(-2 * SIZE, 0)
        ctx.close_path()

        pass




player = Player(0, 0)

v = Vector(3, 6, 77)
print(v)
print(v * 2)

player.a = pi / 4


scene = Scene()

scene.draw_pixel(0, 0, 0)

