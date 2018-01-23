shader_type spatial;
render_mode blend_mix;
uniform sampler2D normal_tex;

void fragment()
{
	ALBEDO = COLOR.rgb;
	ALPHA = COLOR.a;
}