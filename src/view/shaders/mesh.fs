#version 150

uniform mat4 view;             // View matrix
uniform mat4 proj;             // Projection matrix
uniform mat4 inverse_rotation;
uniform mat4 shadow_view_old;
uniform vec4 fixed_color;      // Fixed color for overriding
uniform vec3 light_position_eye; // Light position in eye space
uniform sampler2D tex;         // Texture sampler
uniform float specular_exponent; // Unused in this basic shader
uniform float lighting_factor; // Factor to control lighting contribution
uniform float texture_factor;  // Factor to control texture contribution

in vec3 position_eye;          // Fragment position in eye space
in vec3 normal_eye;            // Normal at the fragment in eye space
in vec2 texcoordi;             // Texture coordinates
in vec4 Kdi;                   // Diffuse reflectivity
in vec4 Kai;                   // Ambient reflectivity

vec3 Ls = vec3(1, 1, 1);       // Specular light color (unused here)
vec3 Ld = vec3(1, 1, 1);       // Diffuse light color
vec3 La = vec3(1, 1, 1);       // Ambient light color

out vec4 outColor;             // Final output color

void main() {
    // Invert the view matrix to transform the light position to world space
    mat4 view_inverse = inverse(view);
    vec3 light_position_world = (view_inverse * vec4(light_position_eye, 1.0)).xyz;

    // Transform the fragment position and normal to world space
    vec3 position_world = (view_inverse * vec4(position_eye, 1.0)).xyz;
    vec3 normal_world = normalize(mat3(view_inverse) * normal_eye);

    // Compute the direction to the light in world space
    vec3 L = normalize(light_position_world - position_world);

    // Diffuse component (Lambertian reflection)
    float NdotL = max(dot(normal_world, L), 0.0); // Clamp to [0, 1]
    vec3 diffuse = Ld * vec3(Kdi) * NdotL;

    // Ambient component
    vec3 ambient = La * vec3(Kai);

    // Combine ambient and diffuse lighting
    vec3 lighting = ambient + lighting_factor * diffuse;

    // Add texture (if enabled)
    vec4 texColor = texture(tex, texcoordi);
    vec3 baseColor = mix(vec3(Kdi), texColor.rgb, texture_factor);

    // Final color output
    outColor = vec4(baseColor + diffuse, Kdi.a);

    // Override with fixed color if provided
    if (fixed_color != vec4(0.0)) {
        outColor = fixed_color;
    }
}
