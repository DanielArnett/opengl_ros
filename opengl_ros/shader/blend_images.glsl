#version 450 core
uniform vec2 resolution;
uniform sampler2D texture; //Assuming sRGB texture; converted to linear RGB by GPU automatically
uniform sampler2D secondTexture;
in vec4 gl_FragCoord;
out vec4 fragColor;

void main(void)
{
    float u = gl_FragCoord.x / resolution.x;
    float v = gl_FragCoord.y / resolution.y;

    vec4 t0 = texture2D(texture, vec2(u,v));
    vec4 t1 = texture2D(secondTexture, vec2(u,v));
    fragColor = (1.0 - t1.a) * t0 + t1.a * t1;
    fragColor.rgb = sqrt(fragColor.rgb);
}
