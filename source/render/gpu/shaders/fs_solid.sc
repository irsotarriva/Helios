$input v_normal, v_texcoord0, v_view

#include <bgfx_shader.sh>

uniform vec4 u_color;  // rgb: albedo
uniform vec4 u_sun;    // xyz: unit direction to the star (universe axes)
uniform vec4 u_params; // x: share of the light that is the cabin's own; y: strength of the grid;
                       // z: light that reaches a face turned away from the star

void main()
{
	float lambert = max(dot(normalize(v_normal), u_sun.xyz), 0.0);
	// The cabin's light comes from where the viewer is, so faces turned away are dimmer and
	// the edges of a box show.
	float cabin = 0.55 + 0.45 * max(normalize(v_view).z, 0.0);
	float light = mix(u_params.z + (1.0 - u_params.z) * lambert, cabin, u_params.x);
	vec3 colour = u_color.rgb * light;

	// Grid lines where a surface coordinate is a whole number, one pixel wide, fading out
	// where the cells shrink to a few pixels.
	vec2 width = max(fwidth(v_texcoord0), vec2(1.0e-6, 1.0e-6));
	vec2 from_line = (0.5 - abs(fract(v_texcoord0) - 0.5)) / width;
	float on_line = 1.0 - clamp(min(from_line.x, from_line.y) - 0.5, 0.0, 1.0);
	float fade = 1.0 - clamp(4.0 * max(width.x, width.y), 0.0, 1.0);
	colour = mix(colour, colour * 1.7 + vec3(0.02, 0.02, 0.02), u_params.y * on_line * fade);

	gl_FragColor = vec4(colour, 1.0);
}
