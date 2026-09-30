$input v_normal

#include <bgfx_shader.sh>

uniform vec4 u_color; // rgb: albedo
uniform vec4 u_sun;   // xyz: unit direction to the star (universe axes); w: 1 for the star itself

void main()
{
	float lambert = max(dot(normalize(v_normal), u_sun.xyz), 0.0);
	float light = mix(0.04 + 0.96 * lambert, 1.0, u_sun.w);
	gl_FragColor = vec4(u_color.rgb * light, 1.0);
}
