from .rotationConversion import *
from .stateConversions import *
from .mixer import *
from .quaternionFunctions import *


def __getattr__(name):
    # Keep the original plotting API available to the desktop application.
    if name in ("fullprint", "makeFigures"):
        from . import display
        return getattr(display, name)
    if name == "third_PV_animation":
        from .animation_multiBoxMarkers import third_PV_animation
        return third_PV_animation
    raise AttributeError(name)
