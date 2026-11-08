// cgltf's implementation (BRIEFING D44), compiled here once, apart from model.cpp: the static
// analyser then sees cgltf's functions as calls, not as code of ours to follow into.
#define CGLTF_IMPLEMENTATION // NOLINT(cppcoreguidelines-macro-usage): cgltf is told so to define its
                             // functions
#include <cgltf.h>
