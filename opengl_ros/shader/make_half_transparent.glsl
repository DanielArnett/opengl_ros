#version 450 core
uniform vec2 resolution;
uniform sampler2D texture; //Assuming sRGB texture; converted to linear RGB by GPU automatically

in vec4 gl_FragCoord;
out vec4 fragColor;

void main(void)
{
    float u = gl_FragCoord.x / resolution.x;
    float v = gl_FragCoord.y / resolution.y;
    vec3 red = vec3(1.0, 0.0, 0.0);
    if (u > 0.5) {
        fragColor = vec4(red, 1.0);
    }
    else {
        fragColor = vec4(red, 0.0);
    }
}
