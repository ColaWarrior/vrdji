#version 300 es
precision mediump float;
uniform sampler2D diffTexture;
uniform bool useEffect;
uniform vec4 effectColor;
in vec2 v2fCoord;
out vec4 oColor;
void main()
{
   if (useEffect == true) {
      oColor = effectColor;
   } else {
      oColor = texture(diffTexture, v2fCoord);
   }
}