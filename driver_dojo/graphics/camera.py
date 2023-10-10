import pyglet
from pyglet.gl import glMatrixMode, GL_MODELVIEW, glLoadIdentity, glPushMatrix, glPopMatrix


class Camera:
    """ A simple 2D camera that contains the speed and offset."""

    def __init__(self, scroll_speed=1, min_zoom=1, max_zoom=10):
        assert min_zoom <= max_zoom, "Minimum zoom must not be greater than maximum zoom"
        self.scroll_speed = scroll_speed
        self.max_zoom = max_zoom
        self.min_zoom = min_zoom
        self.offset_x = 0
        self.offset_y = 0
        self._zoom = max(min(1, self.max_zoom), self.min_zoom)
        self.rotation = 0

    @property
    def zoom(self):
        return self._zoom

    @zoom.setter
    def zoom(self, value):
        """ Here we set zoom, clamp value to minimum of min_zoom and max of max_zoom."""
        self._zoom = max(min(value, self.max_zoom), self.min_zoom)

    @property
    def position(self):
        """Query the current offset."""
        return self.offset_x, self.offset_y

    @position.setter
    def position(self, value):
        """Set the scroll offset directly."""
        self.offset_x, self.offset_y = value

    def move(self, axis_x, axis_y):
        """ Move axis direction with scroll_speed.
            Example: Move left -> move(-1, 0)
         """
        self.offset_x += self.scroll_speed * axis_x
        self.offset_y += self.scroll_speed * axis_y

    def begin(self):
        # Set the current camera offset so you can draw your scene.
        # Translate using the zoom and the offset.
        pyglet.gl.glTranslatef(-self.offset_x * self._zoom, -self.offset_y * self._zoom, 0)

        # Scale by zoom level.
        pyglet.gl.glScalef(self._zoom, self._zoom, 1)

    def end(self):
        # Since this is a matrix, you will need to reverse the translation after rendering otherwise
        # it will multiply the current offset every draw update pushing it further and further away.

        # Reverse scale, since that was the last transform.
        pyglet.gl.glScalef(1 / self._zoom, 1 / self._zoom, 1)

        # Reverse translate.
        pyglet.gl.glTranslatef(self.offset_x * self._zoom, self.offset_y * self._zoom, 0)

    def __enter__(self):
        self.begin()

    def __exit__(self, exception_type, exception_value, traceback):
        self.end()


class CenteredCamera(Camera):
    """A simple 2D camera class. 0, 0 will be the centre of the screen, as opposed to the bottom left."""

    def __init__(self, window: pyglet.window.Window, *args, **kwargs):
        self.window = window
        super().__init__(*args, **kwargs)

    def begin(self):
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        pyglet.gl.glTranslatef(
            self.window.width//2,
            self.window.height//2,
            0
        )  # 4) Move ego to center of window
        pyglet.gl.glTranslatef(
            10, 10, 0
        )
        pyglet.gl.glRotatef(self.rotation + 90, 0, 0, 1)  # 3) Rotate to a) align with x-axis (-rotation) and b) face 90 degree upwards -> faces upward
        pyglet.gl.glTranslatef(-self.offset_x * self.zoom, -self.offset_y * self.zoom, 0)  # 2) Move ego to lower left corner, respecting zoom
        pyglet.gl.glScalef(self._zoom, self._zoom, 1)  # 1) Make bigger

    def end(self):
        pyglet.gl.glScalef(1/self._zoom, 1/self._zoom, 1)  # 1) Make smaller
        pyglet.gl.glTranslatef(self.offset_x * self.zoom, self.offset_y * self.zoom, 0)  # 2) Move ego to its original camera position
        pyglet.gl.glRotatef(-self.rotation - 90, 0, 0, 1)  # 2) Rotate back
        # pyglet.gl.glTranslatef(
        #     -10, -10, 0
        # )
        pyglet.gl.glTranslatef(
            -self.window.width//2,
            -self.window.height//2,
            0
        )  # 1) Move ego to lower left (0, 0, 0)
        glPopMatrix()

    # def begin(self):
    #     x = -self.window.width//2/self._zoom + self.offset_x
    #     y = -self.window.height//2/self._zoom + self.offset_y
    #
    #     pyglet.gl.glTranslatef(-x * self._zoom, -y * self._zoom, 0)
    #     pyglet.gl.glScalef(self._zoom, self._zoom, 1)
    #
    # def end(self):
    #     x = -self.window.width//2/self._zoom + self.offset_x
    #     y = -self.window.height//2/self._zoom + self.offset_y
    #
    #     pyglet.gl.glScalef(1 / self._zoom, 1 / self._zoom, 1)
    #     pyglet.gl.glTranslatef(x * self._zoom, y * self._zoom, 0)
