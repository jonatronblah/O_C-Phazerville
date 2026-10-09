"""C++ emission for Phazerville `HemisphereAudioApplet` applets.

Pure code generation: no I/O, fully unit-testable offline. The entry point is
:func:`generate_applet`, which turns a :class:`~ocapp_agent.models.ChainModel`
into compilable firmware source plus a registration snippet.
"""

from ocapp_agent.codegen.generator import (
    MAX_APPLET_NAME,
    cpp_strhash,
    generate_applet,
    validate_name,
    waveshaper_table,
)

__all__ = [
    "MAX_APPLET_NAME",
    "cpp_strhash",
    "generate_applet",
    "validate_name",
    "waveshaper_table",
]
