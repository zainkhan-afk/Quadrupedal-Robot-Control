        ��  ��                  �   (   G L S L   ���       0 	        #version 150

uniform mat4 ciModelViewProjection;

in vec4 ciPosition;
in vec4 ciColor;

out vec4 color;

void main() {
    gl_Position = ciModelViewProjection * ciPosition;
    color = ciColor;
}]   (   G L S L   ���       0 	        #version 150

in vec4 color;

out vec4 oColor;

void main() {
    oColor = color;
}
   