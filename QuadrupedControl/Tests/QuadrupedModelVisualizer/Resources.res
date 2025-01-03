        ��  ��                  �  (   G L S L   ���       0 	        #version 150

uniform mat4	ciModelViewProjection;
uniform mat3	ciNormalMatrix;

in vec4		ciPosition;
//in vec2		ciTexCoord0;
in vec3		ciNormal;
in vec4		ciColor;

out vec4 outCoord;
//out highp vec2	TexCoord;
out lowp vec4	Color;
out highp vec3	Normal;

void main(void)
{
	gl_Position = ciModelViewProjection * ciPosition;
	outCoord = ciModelViewProjection * ciPosition;
	Color = ciColor;
	//TexCoord = ciTexCoord0;
	Normal = ciNormalMatrix * ciNormal;
}
    (   G L S L   ���       0 	        #version 150

//uniform sampler2D uTex0;

in vec4		outCoord;
in vec4		Color;
in vec3		Normal;
//in vec2		TexCoord;

out vec4 	oColor;

void main(void)
{
	vec3 normal = normalize(Normal);
	vec3 lightPos = vec3(4, 4, 2);
	vec3 lightColor = vec3(1, 1, 1);
	vec3 ambientColor = vec3(0.5, 0.5, 0.5);

	vec3 lightDir = normalize(lightPos - outCoord.xyz);

	float diff = max(dot(normal, lightDir), 0.0);

	vec3 diffuse = diff * lightColor;

	oColor = vec4((ambientColor + diffuse)*Color.xyz, 1.0);
}   