$input a_position
$output v_normal

// A unit sphere: the position is also the normal. The model matrix is rotation · radius plus
// the camera-relative centre (floating origin), so v_normal only needs the rotation part.
#include <bgfx_shader.sh>

void main()
{
	gl_Position = mul(u_modelViewProj, vec4(a_position, 1.0));
	v_normal = normalize(mul(u_model[0], vec4(a_position, 0.0)).xyz);
}
