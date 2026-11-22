// stb_image, public domain, as bgfx's bimg carries it (bimg/3rdparty/stb): it decodes the
// pictures of models (BRIEFING D48). Only PNG and JPEG, from memory. Third-party code, built
// without the project's warnings and lint.
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#define STBI_NO_STDIO
#include <stb_image.h>
