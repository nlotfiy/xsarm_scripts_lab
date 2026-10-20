"""
A key that is seen whatever window has the focus (X11), for the software e-stop.

Uses Xlib's key grab through ctypes, so nothing has to be installed. While the grab is held,
that key goes to this program only, from every window on the display.
"""

import ctypes
import threading

ANY_MODIFIER = 1 << 15     # with or without Shift, Ctrl, Num Lock, Caps Lock...
GRAB_MODE_ASYNC = 1
KEY_PRESS = 2
BAD_ACCESS = 10            # the key is grabbed by another program already


class XEvent(ctypes.Union):
    _fields_ = [('type', ctypes.c_int), ('pad', ctypes.c_long * 24)]


ERROR_HANDLER = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p)


class GlobalKey:
    """Call `on_press` whenever `key` is pressed, in any window of the X display."""

    def __init__(self, key, on_press, display=None):
        """
        Grab the key.

        :param key: X keysym name, such as 'F12' or 'Pause'
        :param on_press: called, from a thread of this object's own, at every press
        :param display: (optional) X display name; the DISPLAY environment variable if None
        :raises RuntimeError: if there is no X display, the key is unknown, or another
            program holds it
        """
        self._x = ctypes.cdll.LoadLibrary('libX11.so.6')
        self._x.XOpenDisplay.restype = ctypes.c_void_p
        self._x.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self._x.XDefaultRootWindow.restype = ctypes.c_ulong
        self._x.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        self._x.XStringToKeysym.restype = ctypes.c_ulong
        self._x.XStringToKeysym.argtypes = [ctypes.c_char_p]
        self._x.XKeysymToKeycode.restype = ctypes.c_ubyte
        self._x.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self._x.XGrabKey.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_uint,
                                     ctypes.c_ulong, ctypes.c_int, ctypes.c_int, ctypes.c_int]
        self._x.XUngrabKey.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_uint,
                                       ctypes.c_ulong]
        self._x.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self._x.XNextEvent.argtypes = [ctypes.c_void_p, ctypes.POINTER(XEvent)]
        self._x.XPending.argtypes = [ctypes.c_void_p]
        self._x.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self._x.XSetErrorHandler.argtypes = [ERROR_HANDLER]
        self._x.XSetErrorHandler.restype = ERROR_HANDLER

        self._display = self._x.XOpenDisplay(display.encode() if display else None)
        if not self._display:
            raise RuntimeError('no X display to grab the key on')
        keysym = self._x.XStringToKeysym(key.encode())
        self._keycode = self._x.XKeysymToKeycode(self._display, keysym) if keysym else 0
        if not self._keycode:
            self._x.XCloseDisplay(self._display)
            raise RuntimeError(f"'{key}' is not a key on this keyboard")
        self._root = self._x.XDefaultRootWindow(self._display)

        errors = []
        self._handler = ERROR_HANDLER(
            lambda display, event: errors.append(ctypes.c_ubyte.from_address(event + 32).value)
            or 0)   # XErrorEvent.error_code
        previous = self._x.XSetErrorHandler(self._handler)
        self._x.XGrabKey(self._display, self._keycode, ANY_MODIFIER, self._root, 1,
                         GRAB_MODE_ASYNC, GRAB_MODE_ASYNC)
        self._x.XSync(self._display, 0)
        self._x.XSetErrorHandler(previous)
        if errors:
            self._x.XCloseDisplay(self._display)
            raise RuntimeError(f"'{key}' is taken by another program"
                               if BAD_ACCESS in errors else f"could not grab '{key}'")
        self.key, self._on_press = key, on_press
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def _listen(self):
        event = XEvent()
        while not self._stop.is_set():
            if not self._x.XPending(self._display):
                self._stop.wait(0.01)
                continue
            self._x.XNextEvent(self._display, ctypes.byref(event))
            if event.type == KEY_PRESS:
                self._on_press()

    def close(self):
        """Give the key back to the other programs."""
        self._stop.set()
        self._thread.join()
        self._x.XUngrabKey(self._display, self._keycode, ANY_MODIFIER, self._root)
        self._x.XSync(self._display, 0)
        self._x.XCloseDisplay(self._display)
