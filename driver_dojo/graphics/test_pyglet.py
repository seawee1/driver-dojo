import pyglet
from pyglet.window import key
from pyglet.gl import glTranslatef, glRotatef, GL_MODELVIEW, glMatrixMode, glLoadIdentity


def movement(keys):
    glMatrixMode(GL_MODELVIEW)
    #glLoadIdentity()
    global x, y, angle
    dx = 5
    da = 20
    if keys[key.W]:
        glTranslatef(0,dx,0)
        y += dx
    if keys[key.S]:
        glTranslatef(0,-dx,0)
        y -= dx
    if keys[key.A]:
        glTranslatef(-dx,0,0)
        x -= dx
    if keys[key.D]:
        glTranslatef(dx,0,0)
        x += dx
    if keys[key.UP]:
        glRotatef(da, 0, 0, 1)
        angle += da
    if keys[key.DOWN]:
        glRotatef(-da, 0, 0, 1)
        angle -= da
    if keys[key.Q]:
        #glPushMatrix()
        #glTranslatef(-x, -y, 0)
        glTranslatef(x, y, 0)
        glRotatef(da, 0, 0, 1)
        glTranslatef(-x, -y, 0)
        #glPopMatrix()
    print(x, y, angle)


def update(dt):
    window.clear()
    label.draw()
    movement(keys)

if __name__ == '__main__':
    window = pyglet.window.Window(height=200, width=200)
    keys = key.KeyStateHandler()
    window.push_handlers(keys)
    x = window.width//2
    y = window.height//2
    angle = 0.0
    label = pyglet.text.Label('Hello, world', font_size=12, x=x, y=y, anchor_x='center', anchor_y='center')

    pyglet.clock.schedule_interval(update,1/5)
    pyglet.app.run()
    # from pyglet.gl import *
    #
    # # Direct OpenGL commands to this window.
    # window = pyglet.window.Window()
    #
    #
    # @window.event
    # def on_draw():
    #     glClear(GL_COLOR_BUFFER_BIT)
    #     glLoadIdentity()
    #     glBegin(GL_TRIANGLES)
    #     glVertex2f(0, 0)
    #     glVertex2f(window.width, 0)
    #     glVertex2f(window.width, window.height)
    #     glEnd()


    pyglet.app.run()