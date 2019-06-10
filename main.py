from math import *
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
# import keyboard

# For my machine to support openCL
# Kernel >= v 4.11 (OK I have v4.15)


# Using Miniconda

# Activating Conda
# source ../../miniconda3/bin/activate

# (one off) Creating the environment
# conda create -n raytracing python=3.7

# Activating the environment
# conda activate raytracing

# Install packages
# conda config --add channels conda-forge
# conda install pycairo
# conda install pygobject

# This one is not in conda-forge
# conda install -c pkgw-forge gtk3

# To capture keyboard input
# conda install -c cogsci pygame


# import pygame


# t = time.time()

# N = 800
# M = 600


# a = np.full((N, M), 1)

# print(a[799][500])

# # for i in range(0, N):
    # # b = []
    # # for j in range(0, M):
        # # b.append(1)

    # # a.append(b)

# t = time.time() - t
# print(t * 1000)

# 1/0



RED = (1, 0, 0)
GREEN = (0, 1, 0)
BLUE = (0, 0, 1)
BLACK = (0, 0, 0)
GREY = (0.3, 0.3, 0.3)
WHITE = (1, 1, 1)
DARKGREEN = (0, 0.5, 0)


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


class Player:
    def __init__(self, a, t, pos = None):
        self.a = a
        self.t = t
        self.pos = pos or Vector(0, 0, 0)

        self.view_speed = 0.06
        self.move_speed = 0.08


# center is a vector
class Sphere:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    # Returns closest intersection point from eye direction to sphere if any
    # e_p (vector) = point E, the eye
    # m_p (vector) = point M, the pixel on the screen
    def intersect_ray(self, e_p, m_p):
        c_p = self.center
        r = self.radius

        m = m_p - e_p

        # m = Vector(m_p.x - e_p.x, m_p.y - e_p.y, m_p.z - e_p.z)

        ec = c_p - e_p

        # a = m.x * m.x + m.y * m.y + m.z * m.z
        # b = - 2 * (m.x * ec.x + m.y * ec.y + m.z * ec.z)
        # c = ec.x ** 2 + ec.y ** 2 + ec.z ** 2 - r ** 2

        a = 1
        b = 1
        c = 1

        a = m.norm_squared()
        b = - 2 * (m ** ec)
        c = ec.norm_squared() - r ** 2

        d = b * b - 4 * a * c
        if d < 0:
            return None

        if d == 0:
            t = -b / (2 * a)
            return e + m * t

        # No need to calculate both roots of the equation
        # We want the closest point that is not behind us
        ds = sqrt(d)

        if - b - ds > 0:
            t = (-b - ds) / (2 * a)
        else:
            t = (-b + ds) / (2 * a)

        i = e_p + m_p * t
        return i


# a, b, c are vectors
class Triangle:
    def __init__(self, a, b, c, a_color = None, b_color = None, c_color = None):
        self.a = a
        self.b = b
        self.c = c

        self.normal = self.get_normal()

        self.a_color = a_color if a_color is not None else DARKGREEN
        self.b_color = b_color if b_color is not None else DARKGREEN
        self.c_color = c_color if c_color is not None else DARKGREEN



    # Solving the three linear equations system
    def intersect_ray(self, e, m):
        av = self.b - self.a
        bv = self.c - self.b
        v = m - e

        matrix = np.array([[av.x, bv.x, - v.x], [av.y, bv.y, - v.y], [av.z, bv.z, - v.z]])
        ret = np.array([e.x - self.a.x, e.y - self.a.y, e.z - self.a.z])

        res = np.linalg.solve(matrix, ret)

        a = res[0]
        b = res[1]
        t = res[2]

        i = e + v * t

        if a > 0 and b > 0 and a <= 1 and b <= a:
            return i
        else:
            return None

    # Should get normal *from a point of view*
    # Not normalized, norm could be different from 1
    def get_normal(self):
        av = self.b - self.a
        bv = self.c - self.b

        # Need to switch entirely to numpy ...
        c = np.cross([av.x, av.y, av.z], [bv.x, bv.y, bv.z])
        return Vector(c[0], c[1], c[2])


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
        # Normalized scalar product
        intensity = - l_v ** normal * (1 / sqrt(normal.norm_squared() * l_v.norm_squared()))
        return intensity if intensity > 0 else 0

class Method:
    RASTERIZING = "rasterizing"
    RAYTRACING = "raytracing"

class Scene:
    # resolution controls image quality (and hence calculations required), needs to be >= 1
    def __init__(self, resolution=1, method=Method.RASTERIZING):
        # Virtual display in front of observer eyes
        # Meters
        self.display_d = 1
        self.display_W = 1
        self.display_H = 0.8

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
        self.method = method

        # Objects and lights
        self.spheres = []
        self.light = None
        self.triangles = []


    def set_player(self, player):
        self.player = player


    def add_sphere(self, sphere):
        self.spheres.append(sphere)


    def add_triangle(self, triangle):
        self.triangles.append(triangle)


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
        p = self.player

        self.drawingarea.queue_draw()
        return

        while(True):
            # Request animation frame
            self.drawingarea.queue_draw()

            key = keyboard.read_key()
            print(key)

            if key == "left":
                p.a += p.view_speed

            if key == "right":
                p.a -= p.view_speed

            if key == "up":
                p.t += p.view_speed

            if key == "down":
                p.t -= p.view_speed

            if key == "q":
                p.pos.x += p.move_speed * sin(p.a) * (-1)
                p.pos.y += p.move_speed * cos(p.a)

            if key == "d":
                p.pos.x += p.move_speed * sin(p.a)
                p.pos.y += p.move_speed * cos(p.a) * (-1)

            time.sleep(0.2)


    # Color is a 3 numbers tuple or list
    def draw_pixel(self, ctx, x, y, color):
        ctx.rectangle(x, self.screen_H - y, self.resolution, self.resolution);
        ctx.set_source_rgb(*color);
        ctx.fill();


    def fill_canvas(self, ctx, color):
        ctx.rectangle(0, 0, self.screen_W, self.screen_H);
        ctx.set_source_rgb(*color);
        ctx.fill();

    def draw_frame(self, da, ctx):
        if self.method == Method.RAYTRACING:
            self.draw_frame_raytracing(da, ctx)
        else:
            self.draw_frame_rasterize(da, ctx)

    # Screen is cleared right before this function gets called
    def draw_frame_raytracing(self, da, ctx):
        e = time.time()

        t_calc = 0
        t_render = 0

        self.fill_canvas(ctx, (0, 0, 0))

        v, w, h = calc_vectors(self.player.a, self.player.t)
        e_p = self.player.pos

        # Bottom left pixel of virtual display
        m_p = self.player.pos + v * self.display_d
        m_p += h * (self.display_H * (-0.5))
        m_p += w * (self.display_W * (-0.5))

        delta_h = h * (self.display_H * self.resolution / self.screen_H)
        delta_w = w * (self.display_W * self.resolution / self.screen_W)

        for x in range(0, self.screen_W, self.resolution):
            reset_h = 0

            for y in range(0, self.screen_H, self.resolution):
                # Calculate pixel color
                t = time.time()

                color = None

                # Should handle visibility problem ...

                for sphere in self.spheres:
                    i = sphere.intersect_ray(e_p, m_p)

                    if i:
                        normal = i - sphere.center
                        intensity = self.light.lambert(i, normal)
                        color = (0, intensity, 0)

                for triangle in self.triangles:
                    i = triangle.intersect_ray(e_p, m_p)

                    if i:
                        intensity = self.light.lambert(i, triangle.normal)
                        color = (0, intensity, 0)


                t_calc += time.time() - t

                # Draw pixel
                t = time.time()

                if color:
                    self.draw_pixel(ctx, x, y, color)

                t_calc += time.time() - t

                # Move virtual pixel up by 1px
                m_p += delta_h
                reset_h += 1

            # Move virtual pixel left by 1px and put it on the bottom of the screen
            m_p -= delta_h * reset_h
            m_p += delta_w

        # Time drawing of frame
        # Note that timing the calculation vs render steps does add overhead
        e = time.time() - e
        fps = 1 / e
        print(f"FPS: {floor(fps)} ; frame time (ms): {floor(1000 * e)} ; calc time (ms): {floor(1000 * t_calc)} ; render time (ms): {floor(1000 * t_render)}")

    # For this scene's player
    # and a pixel on the screen, return a Vector representing
    # this pixel on the virtual display
    # v, w, h are the direction vectors
    # Not used anymore, calculation directly done within the loop
    def get_point(self, screen_x, screen_y, v, w, h):
        res = self.player.pos
        res += v * self.display_d
        res += w * (self.display_W * (screen_x / self.screen_W - 0.5))
        res += h * (self.display_H * (screen_y / self.screen_H - 0.5))
        return res


    def draw_frame_rasterize(self, da, ctx):
        draw_time = time.time()
        self.fill_canvas(ctx, (0.3, 0.3, 0.3))
        pixels = np.full((self.screen_W, self.screen_H), None)
        z_buffer = np.full((self.screen_W, self.screen_H), inf)

        v_v, w_v, h_v = calc_vectors(self.player.a, self.player.t)
        e = self.player.pos
        bl = self.get_point(0, 0, v_v, w_v, h_v)

        for triangle in self.triangles:
            a = self.find_display_pixel(e, triangle.a, bl, w_v, h_v)
            b = self.find_display_pixel(e, triangle.b, bl, w_v, h_v)
            c = self.find_display_pixel(e, triangle.c, bl, w_v, h_v)

            ab = (b[0] - a[0], b[1] - a[1])
            ac = (c[0] - a[0], c[1] - a[1])

            div = ab[1] * ac[0] - ab[0] * ac[1]

            # This means ab and ac are parallel, hence triangle should not be displayed at all (it's a zero-width line)
            if div == 0:
                continue

            # Rough boxing
            min_x = min(a[0], b[0], c[0])
            max_x = max(a[0], b[0], c[0])
            min_y = min(a[1], b[1], c[1])
            max_y = max(a[1], b[1], c[1])

            if min_x < 0:
                min_x = 0

            if min_y < 0:
                min_y = 0

            if max_x >= self.screen_W:
                max_x = self.screen_W - 1

            if max_y >= self.screen_H:
                max_y = self.screen_H - 1

            # Vertices z-buffers
            za = self.find_z_index(e, triangle.a, v_v, w_v, h_v)
            zb = self.find_z_index(e, triangle.b, v_v, w_v, h_v)
            zc = self.find_z_index(e, triangle.c, v_v, w_v, h_v)

            # Vertices lighting
            light_a = self.light.lambert(triangle.a, triangle.normal)
            light_b = self.light.lambert(triangle.b, triangle.normal)
            light_c = self.light.lambert(triangle.c, triangle.normal)

            for x in range(min_x, max_x + 1, self.resolution):
                for y in range(min_y, max_y + 1, self.resolution):
                    am = (x - a[0], y - a[1])
                    gamma = (ab[1] * am[0] - ab[0] * am[1]) / div

                    # Vector AB can't be null or div would be null
                    if ab[0] != 0:
                        beta = (am[0] - ac[0] * gamma) / ab[0]
                    else:
                        beta = (am[1] - ac[1] * gamma) / ab[1]

                    if beta >= 0 and gamma >= 0 and gamma <= 1 - beta:
                        # Barycentric coefficients are alpha for A, beta for B and gamma for C
                        alpha = 1 - beta - gamma

                        # Calculate z-buffer and modify frame buffer only if closer to screen
                        z = 1 / (alpha / za + beta / zb + gamma / zc)
                        if z < z_buffer[x][y]:
                            z_buffer[x][y] = z

                            # Coloring by depth
                            # Temporary test formula ; to be interpolated instead
                            # intensity = (8 - z_buffer[x][y]) / 3
                            # color = (0, intensity, 0)

                            # Texturing
                            # Texture not aligned to triangle but seems to work
                            xp = z_buffer[x][y] * (alpha * triangle.a.x / za + beta * triangle.b.x / zb + gamma * triangle.c.x / zc)
                            yp = z_buffer[x][y] * (alpha * triangle.a.y / za + beta * triangle.b.y / zb + gamma * triangle.c.y / zc)

                            xp = floor(5 * xp)
                            yp = floor(5 * yp)

                            if (xp + yp) % 2 == 0:
                                color = WHITE
                            else:
                                color = BLACK

                            # Color interpolation
                            # r = z_buffer[x][y] * (alpha * triangle.a_color[0] / za + beta * triangle.b_color[0] / zb + gamma * triangle.c_color[0] / zc)
                            # g = z_buffer[x][y] * (alpha * triangle.a_color[1] / za + beta * triangle.b_color[1] / zb + gamma * triangle.c_color[1] / zc)
                            # b = z_buffer[x][y] * (alpha * triangle.a_color[2] / za + beta * triangle.b_color[2] / zb + gamma * triangle.c_color[2] / zc)
                            # color = (r, g, b)

                            # Lighting interpolation
                            light = z_buffer[x][y] * (alpha * light_a / za + beta * light_b / zb + gamma * light_c / zc)
                            color = (0, light, 0)

                            # Linear color interpolation (BAD)
                            # r = alpha * triangle.a_color[0] + beta * triangle.b_color[0] + gamma * triangle.c_color[0]
                            # g = alpha * triangle.a_color[1] + beta * triangle.b_color[1] + gamma * triangle.c_color[1]
                            # b = alpha * triangle.a_color[2] + beta * triangle.b_color[2] + gamma * triangle.c_color[2]
                            # color = (r, g, b)

                            # Linear texture interpolation (BAD)
                            # xp = alpha * triangle.a.x + beta * triangle.b.x + gamma * triangle.c.x
                            # yp = alpha * triangle.a.y + beta * triangle.b.y + gamma * triangle.c.y

                            # xp = floor(5 * xp)
                            # yp = floor(5 * yp)

                            # if (xp + yp) % 2 == 0:
                                # color = WHITE
                            # else:
                                # color = BLACK

                            # Setting pixel color
                            pixels[x][y] = color


            # Actually draw pixels
            for x in range(0, self.screen_W):
                for y in range(0, self.screen_H):
                    if pixels[x][y] is not None:
                        self.draw_pixel(ctx, x, y, pixels[x][y])


        draw_time = time.time() - draw_time
        fps = 1 / draw_time
        print(f"FPS: {floor(fps)} ; frame time (ms): {floor(1000 * draw_time)}")


    # Virtual display is defined by its bottom left corner bl, its width vector w_v and its height vector h_v
    # e is the eye
    # m is the real point for which we want to find the corresponding pixel
    def find_display_pixel(self, e, m, bl, w_v, h_v):
        me_v = e - m
        matrix = np.array([[w_v.x, h_v.x, me_v.x], [w_v.y, h_v.y, me_v.y], [w_v.z, h_v.z, me_v.z]])
        ret = np.array([e.x - bl.x, e.y - bl.y, e.z - bl.z])
        res = np.linalg.solve(matrix, ret)

        w = res[0]
        h = res[1]
        t = res[2]

        # Ignoring case where triangle is fully or partially behind observer
        # if t <= 0:
            # return None

        # Ignoring case where point is out of the screen (w or h not in [0, 1])

        # Assuming w_v and h_v have norm 1
        x = int(round(self.screen_W * w / self.display_W))
        y = int(round(self.screen_H * h / self.display_H))

        return (x, y)

    # Assuming v_v has norm 1
    def find_z_index(self, e, m, v_v, w_v, h_v):
        em_v = m - e
        matrix = np.array([[v_v.x, w_v.x, h_v.x], [v_v.y, w_v.y, h_v.y], [v_v.z, w_v.z, h_v.z]])
        ret = np.array([em_v.x, em_v.y, em_v.z])

        res = np.linalg.solve(matrix, ret)

        v = res[0]

        return v


player = Player(0, 0, Vector(0, 0, 0))
scene = Scene(resolution=1, method=Method.RASTERIZING)
# scene = Scene(resolution=3, method=Method.RAYTRACING)
scene.set_player(player)

s1 = Sphere(Vector(5, 5, 1), 0.6)
s2 = Sphere(Vector(3, 5, 0), 0.8)
light = Light(Vector(0, 0, 0))
# t1 = Triangle(Vector(5, 2, 0), Vector(5, -2, 0), Vector(8, 0, 3))
t1 = Triangle(Vector(6, 2, 0), Vector(3.2, -1.5, -0.5), Vector(8, -0.5, 3), a_color = RED, b_color = GREEN, c_color = BLUE)
t2 = Triangle(Vector(4, 2, 0), Vector(6, -1.5, -0.5), Vector(7, 2, 3), a_color = RED, b_color = GREEN, c_color = BLUE)

scene.add_sphere(s1)
scene.add_sphere(s2)
scene.add_triangle(t1)
scene.add_triangle(t2)
scene.add_light(light)


scene.start()

