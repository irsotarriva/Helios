$input a_position, a_normal, a_texcoord0
$output v_normal, v_texcoord0, v_view

// A solid of the flight view: a part, a piece of the cockpit, the ground. The model matrix is
// a rotation with a scale along each of the shape's own axes, plus the camera-relative
// position (floating origin). The normals of the unit shapes lie along those axes or across
// the one that is scaled differently, so the same matrix turns them correctly.
#include <bgfx_shader.sh>

void main()
{
	gl_Position = mul(u_modelViewProj, vec4(a_position, 1.0));
	vec3 normal = normalize(mul(u_model[0], vec4(a_normal, 0.0)).xyz);
	v_normal = normal;
	v_view = mul(u_view, vec4(normal, 0.0)).xyz;
	v_texcoord0 = a_texcoord0;
}
