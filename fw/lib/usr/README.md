# libusr
#### Common User Code Library

This is the go-to place for any utilities or headers
intended to be used across a multitude of projects.

The canonical example is `<usr/type.h>`, which pulls
in standard library headers and defines commonly used
custom types such as `u8` and `u32`.

To use: link to target `usr` for common headers.
Additional components can be found by perusing the
project's subfolders.