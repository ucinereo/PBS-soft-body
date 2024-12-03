#version 150
uniform mat4 view;
uniform mat4 proj;
uniform mat4 inverse_rotation;
uniform mat4 shadow_view_old;
uniform vec4 fixed_color;
in vec3 position_eye;
in vec3 normal_eye;
uniform bool is_directional_light;
uniform bool is_shadow_mapping;
uniform bool shadow_pass;
uniform vec3 light_position_eye;
vec3 Ls = vec3 (1, 1, 1);
vec3 Ld = vec3 (1, 1, 1);
vec3 La = vec3 (1, 1, 1);
in vec4 Ksi;
in vec4 Kdi;
in vec4 Kai;
in vec2 texcoordi;
uniform sampler2D tex;
uniform float specular_exponent;
uniform float lighting_factor;
uniform float texture_factor;
uniform float matcap_factor;
uniform float double_sided;

uniform sampler2D shadow_tex;
in vec4 position_shadow;

out vec4 outColor;
void main()
{
  if(shadow_pass)
  {
    // Would it be better to have a separate no-op frag shader?
    // outColor = vec4(0.56,0.85,0.77,1.);
    outColor = vec4(1.0, 1.0, 1.0, 1.0);
    return;
  }
  // If is_directional_light then assume normalized
  vec3 direction_to_light_eye = light_position_eye;
  if(! is_directional_light)
  {
    vec3 vector_to_light_eye = light_position_eye - position_eye;
    direction_to_light_eye = normalize(vector_to_light_eye);
  }
  float shadow = 1.0;
  if(is_shadow_mapping)
  {
    vec3 shadow_pos = (position_shadow.xyz / position_shadow.w) * 0.5 + 0.5; 
    float currentDepth = shadow_pos.z;
    //float bias = 0.005;
    float ddd = max(dot(normalize(normal_eye), direction_to_light_eye),0);
    float bias = max(0.02 * (1.0 - ddd), 0.005);  
    // 5-point stencil
    if(shadow_pos.z < 1.0)
    {
      float closestDepth = texture( shadow_tex , shadow_pos.xy).r;
      shadow = currentDepth - bias >= closestDepth ? 0.0 : 1.0;  
      vec2 texelSize = 1.0 / textureSize(shadow_tex, 0);
      for(int x = -1; x <= 1; x+=2)
      {
        for(int y = -1; y <= 1; y+=2)
        {
          float pcfDepth = texture(shadow_tex,  shadow_pos.xy + vec2(x, y) * texelSize).r; 
          shadow += currentDepth - bias >= pcfDepth ? 0.0 : 1.0;        
        }    
      }
      shadow /= 5.0;
    }
  }

  if(matcap_factor == 1.0f)
  {
    vec2 uv = normalize(normal_eye).xy * 0.5 + 0.5;
    outColor = mix(Kai,texture(tex, uv),shadow);
  }else
  {
    vec3 Ia = La * vec3(Kai);    // ambient intensity

    float dot_prod = dot (direction_to_light_eye, normalize(normal_eye));
    float clamped_dot_prod = abs(max (dot_prod, -double_sided));
    vec3 Id = Ld * vec3(Kdi) * clamped_dot_prod;    // Diffuse intensity

    vec3 reflection_eye = reflect (-direction_to_light_eye, normalize(normal_eye));
    vec3 surface_to_viewer_eye = normalize (-position_eye);
    float dot_prod_specular = dot (reflection_eye, surface_to_viewer_eye);
    dot_prod_specular = float(abs(dot_prod)==dot_prod) * abs(max (dot_prod_specular, -double_sided));
    float specular_factor = pow (dot_prod_specular, specular_exponent);
    vec3 Is = Ls * vec3(Ksi) * specular_factor;    // specular intensity
    vec4 color = vec4(Ia + shadow*(lighting_factor * (Is + Id) + (1.0-lighting_factor) * vec3(Kdi)),(Kai.a+Ksi.a+Kdi.a)/3);
    outColor = mix(vec4(1,1,1,1), texture(tex, texcoordi), texture_factor) * color;
    if (fixed_color != vec4(0.0)) outColor = fixed_color;
  }
}