$input v_texcoord0

#include <bgfx_shader.sh>

// The picture the screen shows: it gives its own light.
SAMPLER2D(s_screen, 0);

void main()
{
	gl_FragColor = vec4(texture2D(s_screen, v_texcoord0).rgb, 1.0);
}
