import cairo
import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

SIZE = 30

import time
from threading import Timer


def draw_shapes(ctx, x, y):
    ctx.save()

    ctx.translate(x + SIZE, y + SIZE)
    ctx.new_path()
    ctx.translate(3 * SIZE, 0)
    ctx.move_to(0, 0)
    ctx.rel_line_to(2 * SIZE, 0)
    ctx.rel_line_to(0, 2 * SIZE)
    ctx.rel_line_to(-2 * SIZE, 0)
    ctx.close_path()
    ctx.fill()

    ctx.restore()


def draw(da, ctx):
    print("Draw")

    SIZE = 30
    x = 0
    y = 12

    ctx.set_line_width(SIZE / 4)
    ctx.set_tolerance(0.1)

    ctx.set_source_rgb(0, 0, 0)

    ctx.save()

    ctx.translate(x + SIZE, y + SIZE)
    ctx.new_path()
    ctx.translate(3 * SIZE, 0)
    ctx.move_to(0, 0)
    ctx.rel_line_to(2 * SIZE, 0)
    ctx.rel_line_to(0, 2 * SIZE)
    ctx.rel_line_to(-2 * SIZE, 0)
    ctx.close_path()
    ctx.fill()

    ctx.restore()



    # draw_shapes(ctx, 0, 12 * SIZE)

    # ctx.set_source_rgb(1, 0, 0)
    # draw_shapes(ctx, 0, 15 * SIZE)

    # t = Timer(3, another, [da, ctx])
    # t.start()

    # time.sleep(3)
    # another(da, ctx)


def another(da, ctx):
    print("Another")

    ctx.set_source_rgb(0, 0.5, 0)
    draw_shapes(ctx, 4 * SIZE, 12 * SIZE)


def main():
    win = Gtk.Window()
    win.connect('destroy', lambda w: Gtk.main_quit())
    win.set_default_size(450, 550)

    drawingarea = Gtk.DrawingArea()
    win.add(drawingarea)
    drawingarea.connect('draw', draw)

    win.show_all()
    Gtk.main()


main()

