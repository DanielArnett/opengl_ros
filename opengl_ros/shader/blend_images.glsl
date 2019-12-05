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
    vec3 red = vec3(1.0, 0.0, 0.0);
    vec3 green = vec3(0.0, 1.0, 0.0);
    vec4 inputPixel = texture2D(texture, vec2(u,v));
    fragColor = inputPixel + texture2D(secondTexture, vec2(u,v));
    float opacity = inputPixel.w;
    //if (opacity == 1.0) {
    //    fragColor = vec4(red, 1.0);
    //}
    //else {
    //    fragColor = vec4(green, 0.0);
    //}
}
