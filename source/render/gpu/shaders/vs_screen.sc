$input a_position, a_texcoord0
$output v_texcoord0

// The glass of a cockpit's screen (BRIEFING D46), in camera space like the lettering.
#include <bgfx_shader.sh>

void main()
{
	gl_Position = mul(u_viewProj, vec4(a_position, 1.0));
	v_texcoord0 = a_texcoord0;
}
