#version 450
#extension GL_ARB_separate_shader_objects : enable

#include "inc/defines.glsl"
#include "inc/lighting.glsl"

// should be an even number
layout(constant_id = 0) const int MAX_IMPORTANCE_SAMPLES = 32;

layout(binding = 0) uniform CameraUBO {
  mat4 view;
  mat4 proj;
  vec3 eye;
  vec3 viewDir;
  float farDist;
  float nearDist;
} cam;

layout(binding = 1) uniform ShadowLights {
  ShadowLight at[MAX_GLOBAL_LIGHTS];
  uint count;
} lights;

layout(binding = 2) uniform ImportanceSampling {
  vec4 samples[MAX_IMPORTANCE_SAMPLES / 2]; // divide by 2 because each vec2 is a sample
} importance;

// shader control
layout(binding = 3) SHADER_CONTROL_UNIFORM control;

layout(binding = 4) uniform sampler2D inGBuffPosition;
layout(binding = 5) uniform sampler2D inGBuffNormal;
layout(binding = 6) uniform sampler2D inGBuffColor;
layout(binding = 7) uniform sampler2D inBackground;
layout(binding = 8) uniform sampler2D inIrradiance;

layout(binding = 9) uniform sampler2D shadowMap[MAX_GLOBAL_LIGHTS];

layout(location = 0) in vec2 inUV;

layout(location = 0) out vec4 fragColor;

// This function takes the sample (u, v) as the epsilon 1 and epsilon 2
vec3 getSampleDirection(float u, float v, float roughness, vec3 A, vec3 B, vec3 R) {
  float alpha = (roughness * roughness) / 2;

  // skew the points to match the distribution
  v = acos(pow(v, 1.0 / (alpha + 1)));

  float temp = u;
  u = v;
  v = temp;
  
  // convert UV to hemisphere sample
  // this is not correct - somewhere
  float inner = 2 * PI * (.5 - u);
  float sinpv = sin(PI * v);
  vec3 L = vec3(cos(inner) * sinpv, sin(inner) * sinpv, cos(PI * v));

  // rotate
  vec3 w = normalize(L.x * A + L.y * B + L.z * R); // normalizing done to ward off rounding errors
  return w;
}

float getG(vec4 moments, float fragmentDepth) {
  // Code comes from the supplementary paper for Hamburger 4MSM.
  // Converted from HLSL to GLSL.
  vec4 b = moments * (1 - control.momentBias) + control.momentBias * vec4(0.5f, 0.5f, 0.5f, 0.5f);
  vec3 z = vec3(fragmentDepth - control.depthBias, 0, 0);
  
  // Cholesky decomposition
  float L32D22 = -b[0] * b[1] + b[2];
  float D22    = -b[0] * b[0] + b[1];
  float SDV    = -b[1] * b[1] + b[3];
  float D33D22 = dot( vec2(SDV, -L32D22), vec2(D22, L32D22) );
  float InvD22 = 1.0 / D22;
  float L32    = L32D22 * InvD22;
  
  vec3 c = vec3(1.0f, z[0], z[0] * z[0]);
  
  c[1] -= b.x;
  c[2] -= b.y + L32 * c[1];
  
  c[1] *= InvD22;
  c[2] *= D22 / D33D22;
  
  c[1] -= L32 * c[2];
  c[0] -= dot(c.yz, b.xy);
  
  // Quadratic formula solving
  float p = c[1] / c[2];
  float q = c[0] / c[2];
  float D = ( (p * p) / 4.0f ) - q;
  float r = sqrt(D);
  
  z[1] = -(p / 2.0f) - r;
  z[2] = -(p / 2.0f) + r;
  
  // Final calculation of G
  vec4 switchVec = (z[2] < z[0])
    ? vec4(z[1], z[0], 1.0f, 1.0f)
    : (z[1] < z[0])
      ? vec4(z[0], z[1], 0.0f, 1.0f)
      : vec4(0.f, 0.f, 0.f, 0.f);
      
  float quotient = (switchVec[0] * z[2] - b[0] * (switchVec[0] + z[2]) + b[1])
                 / ((z[2] - switchVec[1]) * (z[0] - z[1]));
                 
  return clamp(switchVec[2] + switchVec[3] * quotient, 0.0f, 1.0f);
}

void main() {
  vec4 sampledPos = texture(inGBuffPosition, inUV);
  vec4 sampledColor = texture(inGBuffColor, inUV);
  vec4 sampledNormal = texture(inGBuffNormal, inUV);
  
  float isObject    = sampledPos.w;
  float inRoughness = sampledColor.w;
  float inMetallic  = sampledNormal.w;
  vec3 inPos = sampledPos.xyz;
  vec3 inColor = sampledColor.xyz;
  
  vec3 V = normalize(cam.eye - inPos);

  if(int(isObject) == 1 && control.doGlobalLighting == 1) {
    vec3 N = normalize(sampledNormal.xyz);
    vec3 R = 2 * max(dot(N, V), 0) * N - V;

    mat4 shadowBias = mat4( .5,  0,  0, 0,
                            0, .5,  0, 0,
                            0,  0,  1, 0,
                            .5, .5,  0, 1);
    
    vec3 color = vec3(0, 0, 0);
    for(int i = 0; i < lights.count; ++i) {
      vec4 shadowCoord =  shadowBias * lights.at[i].proj *  lights.at[i].view * vec4(inPos, 1.f);
      
      vec2 shadowIndex = shadowCoord.xy / shadowCoord.w;
      
      if(shadowCoord.w > 0 && shadowIndex.x >= 0 && shadowIndex.y >= 0 && shadowIndex.x <= 1 && shadowIndex.y <= 1) {
        vec4 lightDepth = texture(shadowMap[i], shadowIndex);
        float pixelDepth = shadowCoord.z;
        pixelDepth = (pixelDepth - lights.at[i].nearDist) / (lights.at[i].farDist - lights.at[i].nearDist);
        
        float G = getG(lightDepth, pixelDepth);
        
        if(G < 1 || control.doShadows == 0) {         
          vec3 lightColor = lights.at[i].color;
          vec3 lightPos   = lights.at[i].pos;
          vec3 lightAtten = lights.at[i].atten;
          float lightRad  = lights.at[i].radius * 100;
          
          vec3 pbrColor = computeDirectPBR(lightColor, lightPos, lightAtten, lightRad, inColor, inPos, N, V, inRoughness, inMetallic);
          
          if(control.doShadows == 0)
            G = 0;

          color += (1 - G) * pbrColor;
        }
      }
    }
    
    // Add in IBL:
    if(control.doIBLLighting == 1) {
      // Diffuse:
      vec2 bgColorUV = vec2(.5 - atan(-N.y, -N.x) / (2 * PI), acos(-N.z) / PI);
      vec4 sampledIrradiance = texture(inIrradiance, bgColorUV);
      vec3 IBL_diffuse = computeIBLPBRDiffuse(sampledIrradiance.xyz, inColor) * max(dot(N, V), 0);

      // Specular:
      vec3 A = normalize(vec3(-R.y, R.x, 0)); // cross R w/ Z-axis
      vec3 B = normalize(cross(R, A));
      float sampleCountCoeff = 1.0 / MAX_IMPORTANCE_SAMPLES;

      vec3 IBL_specular = vec3(0, 0, 0);
      for(int i = 0; i < MAX_IMPORTANCE_SAMPLES; ++i) {
        // find specular lighting sample direction
        vec4 sampleVec4 = importance.samples[i / 2];
        vec2 sampleVec2 = i % 2 == 0 ? sampleVec4.xy : sampleVec4.zw;
        vec3 w = getSampleDirection(sampleVec2.x, sampleVec2.y, inRoughness, A, B, R);
        vec2 uv = vec2(.5 - atan(-w.y, -w.x) / (2 * PI), acos(-w.z) / PI);
      
        // TODO: Read from a specific mipmap level

        vec3 H = normalize(w + V);

        float NdotV = max(dot(N, V), 0);
        float NdotL = max(dot(N, w), 0);

        float NDF = DistributionGGX(N, H, inRoughness);
        float G   = GeometrySmith_IBL(NdotV, NdotL, inRoughness);

        vec3 F0 = mix(vec3(0.0), inColor, inMetallic);
        vec3 F  = fresnelSchlick(max(dot(H, V), 0), F0);

        // Note: the D term is not included in the numerator, as it cancels
        // out with the probability distribution, as we say that the
        // probability of each sample is the NDF.
        vec3 numer = G * F;
        float denom = 4.0 * NdotV * NdotL;

        vec3 specular = numer / max(denom, 0.01);

        ivec2 bgSize = textureSize(inBackground, 0);
        float mipLevel = .5 * log2((bgSize.x + bgSize.y) / 2.0) - .5 * log2(NDF);
        vec3 sampledBG = textureLod(inBackground, uv, mipLevel).rgb;

        IBL_specular += sampleCountCoeff * specular * sampledBG * NdotL;
      }

      color += IBL_specular;
    }
    fragColor = vec4(color, 1);
  }
  else {
    // just do the background color
    if(control.enableHDRBackground == 1) {
      vec2 bgColorUV = vec2(.5 - atan(V.y, V.x) / (2 * PI), acos(V.z) / PI);
      vec4 sampledBG = texture(inBackground, bgColorUV);
      fragColor = vec4(sampledBG.xyz, 1);
    }
    else
      fragColor = vec4(0, 0, 0, 1);
  }
}