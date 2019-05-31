from math import cos, sin, pi, sqrt, floor

# sudo apt-get install cairo
# pip install pycairo
import cairo

# sudo apt install libgirepository1.0-dev gcc libcairo2-dev pkg-config python3-dev gir1.2-gtk-3.0
# pip3 install PyGObject
import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

import threading
import time

import random


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


class Scene:
    def __init__(self):
        # Virtual display in front of observer eyes
        # Meters
        self.display_d = 1
        self.display_W = 1
        self.display_H = 0.75

        # Screen (a gtk window)
        self.screen_W = 800
        self.screen_H = 600
        self.ctx = None

        win = Gtk.Window()
        win.connect('destroy', lambda w: Gtk.main_quit())
        win.set_default_size(self.screen_W, self.screen_H)

        self.drawingarea = Gtk.DrawingArea()
        win.add(self.drawingarea)
        self.drawingarea.connect('draw', self.draw_frame)

        win.show_all()


    def set_player(self, player):
        self.player = player


    def start(self):
        thread = threading.Thread(target=self.animate)
        thread.daemon = True
        thread.start()
        # Start Gtk window (blocking call)
        Gtk.main()


    def animate(self):
        while(True):
            # Request animation frame and time it
            t = time.time()
            self.drawingarea.queue_draw()
            e = time.time() - t
            fps = 1 / e
            print(f"FPS: {floor(fps)} ; frame time (ms): {floor(1000 * e)}")
            time.sleep(5)


    # Color is a 3 numbers tuple or list
    def draw_pixel(self, ctx, x, y, color):
        ctx.rectangle(x, self.screen_H - y, 1, 1);
        ctx.set_source_rgb(*color);
        ctx.fill();


    # Screen is cleared right before this function gets called
    def draw_frame(self, da, ctx):
        for x in range(0, self.screen_W):
            for y in range(0, self.screen_H):
                r = random.randrange(0, 100) / 100
                g = random.randrange(0, 100) / 100
                b = random.randrange(0, 100) / 100
                c = (r, g, b)

                self.draw_pixel(ctx, x, y, c)


    # For this scene's player
    # and a pixel on the screen, return a Vector representing
    # this pixel on the virtual display
    def get_point(self, screen_x, screen_y):
        v, w, h = calc_vectors(self.player.a, self.player.t)
        res = self.player.pos
        res += v * self.display_d
        res += w * (self.display_W * (screen_x / self.screen_W - 0.5))
        res += h * (self.display_H * (screen_y / self.screen_H - 0.5))
        return res





player = Player(0, 0)
scene = Scene()
scene.set_player(player)
scene.start()

