#version 450 core
uniform vec2 resolution;
uniform sampler2D texture; //Assuming sRGB texture; converted to linear RGB by GPU automatically
uniform float x, y;
in vec4 gl_FragCoord;
out vec4 fragColor;

void main(void)
{
    float u = gl_FragCoord.x / resolution.x;
    float v = gl_FragCoord.y / resolution.y;
    vec3 black = vec3(0.0, 0.0, 0.0);
    vec3 red = vec3(1.0, 0.0, 0.0);
    vec3 green = vec3(0.0, 1.0, 0.0);
    vec3 color = green;
    if (x < 0.5) {
        color = red;
    }
    float intensity = 1.0 - distance(vec2(u,v), vec2(x,y));
    //color *= intensity;
    fragColor = vec4(color, intensity);
    //if (distance(vec2(u,v), vec2(x,y)) < 0.2) {
    //    fragColor = vec4(red, 1.0);
    //}
    //else {
    //    fragColor = vec4(black, 0.0);
    //}
}
