from math import cos, sin, pi, sqrt, floor
import threading
import time
import random

# sudo apt-get install cairo
# pip install pycairo
import cairo

# sudo apt install libgirepository1.0-dev gcc libcairo2-dev pkg-config python3-dev gir1.2-gtk-3.0
# pip3 install PyGObject
import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

# pip install numpy
# Unused for now
import numpy as np

# pip install keyboard
# Need to be root to execute script
import keyboard

# For my machine to support openCL
# Kernel >= v 4.11 (OK I have v4.15)


# Using Miniconda
# conda config --add channels conda-forge
# conda install pycairo
# conda install pygobject

# This one is not in conda-forge
# conda install -c pkgw-forge gtk3




class Vector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def norm_squared(self):
        return self.x * self.x + self.y * self.y + self.z * self.z

    def norm(self):
        return sqrt(self.norm_squared())

    def normalize(self):
        return self * (1 / self.norm())

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
    w = Vector(-v.y / cos(t), v.x / cos(t), 0) * (-1)
    h = Vector(-sin(t) * cos(a), -sin(t) * sin(a), cos(t))

    return v, w, h



# Returns closest intersection point from eye direction to sphere if any
# e_p (vector) = point E, the eye
# m_p (vector) = point M, the pixel on the screen
def intersect_sphere(e_p, m_p, sphere):
    c_p = sphere.center
    r = sphere.radius

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

    i1 = e_p + m_p * t1
    i2 = e_p + m_p * t2

    if (i1 - e_p).norm() > (i2 - e_p).norm():
        return i2
    else:
        return i1


class Player:
    def __init__(self, a, t, pos = None):
        self.a = a
        self.t = t
        self.pos = pos or Vector(0, 0, 0)


# center is a vector
class Sphere:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius


# pos is a vector
class Light:
    def __init__(self, pos):
        self.pos = pos

    # First test of lighting fitted to the specific sphere we used during tests
    def intensity(self, i):
        d = (i - self.pos).norm()

        _min = sqrt(17) - 0.6
        _max = sqrt(17) + 0.3

        intensity = (_max - d) / (_max - _min) - 0.2
        return intensity

    # normal is the normal vector on surface at point i
    def lambert(self, i, normal):
        l_v = i - self.pos
        l_v = l_v.normalize()

        intensity = - l_v ** normal
        intensity = max(0, intensity)
        return intensity


class Scene:
    # resolution controls image quality (and hence calculations required), needs to be >= 1
    def __init__(self, resolution=1):
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

        # Misc
        self.resolution = resolution

        # Object
        self.spheres = []


    def set_player(self, player):
        self.player = player


    # Later we'll be able to have many spheres
    def add_sphere(self, sphere):
        self.spheres.append(sphere)


    # Later we'll be able to add many lights
    def add_light(self, light):
        self.light = light


    def start(self):
        thread = threading.Thread(target=self.animate)
        thread.daemon = True
        thread.start()
        # Start Gtk window (blocking call)
        Gtk.main()


    def animate(self):
        while(True):
            # Request animation frame
            self.drawingarea.queue_draw()

            key = keyboard.read_key()
            print(key)

            if key == "left":
                self.player.a += 0.06

            if key == "right":
                self.player.a -= 0.06

            if key == "up":
                self.player.t += 0.06

            if key == "down":
                self.player.t -= 0.06

            time.sleep(0.2)


    # Color is a 3 numbers tuple or list
    def draw_pixel(self, ctx, x, y, color):
        ctx.rectangle(x, self.screen_H - y, self.resolution, self.resolution);
        ctx.set_source_rgb(*color);
        ctx.fill();


    # Screen is cleared right before this function gets called
    def draw_frame(self, da, ctx):
        e = time.time()

        t_calc = 0
        t_render = 0

        for x in range(0, self.screen_W, self.resolution):
            for y in range(0, self.screen_H, self.resolution):
                # Calculate pixel color
                t = time.time()

                color = (0, 0, 0)

                e_p = self.player.pos
                m_p = self.get_point(x, y)

                for sphere in self.spheres:
                    i = intersect_sphere(e_p, m_p, sphere)

                    if i:
                        intensity = self.light.intensity(i)

                        normal = i - sphere.center
                        normal = normal.normalize()

                        intensity = self.light.lambert(i, normal)

                        color = (0, intensity, 0)

                t_calc += time.time() - t

                # Draw pixel
                t = time.time()

                self.draw_pixel(ctx, x, y, color)

                t_calc += time.time() - t

        # Time drawing of frame
        # Note that timing the calculation vs render steps does add overhead
        e = time.time() - e
        fps = 1 / e
        print(f"FPS: {floor(fps)} ; frame time (ms): {floor(1000 * e)} ; calc time (ms): {floor(1000 * t_calc)} ; render time (ms): {floor(1000 * t_render)}")

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





player = Player(pi/4, 0, Vector(0, 0, 0))
scene = Scene(resolution=3)
scene.set_player(player)

s1 = Sphere(Vector(5, 5, 1), 0.6)
s2 = Sphere(Vector(3, 5, 0), 0.8)
light = Light(Vector(4, 0, 0))

scene.add_sphere(s1)
scene.add_sphere(s2)
scene.add_light(light)


scene.start()

