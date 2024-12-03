#version 150
uniform mat4 view;
uniform mat4 proj;
uniform mat4 inverse_rotation;
uniform mat4 shadow_view_old;
uniform mat4 normal_matrix;
in vec3 position;
in vec3 normal;
out vec3 position_eye;
out vec3 normal_eye;
in vec4 Ka;
in vec4 Kd;
in vec4 Ks;
in vec2 texcoord;
out vec2 texcoordi;
out vec4 Kai;
out vec4 Kdi;
out vec4 Ksi;
uniform mat4 shadow_view;
uniform mat4 shadow_proj;
uniform bool shadow_pass;
uniform bool is_shadow_mapping;
out vec4 position_shadow;

void main()
{
  position_eye = vec3 (view * vec4 (position, 1.0));
  if(!shadow_pass)
  {
    if(is_shadow_mapping)
    {
      // mat4 tmp_view = shadow_view_old * inverse_rotation * inverse(shadow_view_old) * shadow_view;
      position_shadow = shadow_proj * shadow_view * vec4(position, 1.0);
    }
    normal_eye = normalize(normal);
    Kai = Ka;
    Kdi = Kd;
    Ksi = Ks;
    texcoordi = texcoord;
  }
  gl_Position = proj * vec4 (position_eye, 1.0); //proj * view * vec4(position, 1.0);
}