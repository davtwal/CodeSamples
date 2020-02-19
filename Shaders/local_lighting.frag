#version 450
#extension GL_ARB_separate_shader_objects : enable

#include "inc/defines.glsl"
#include "inc/lighting.glsl"

layout(binding = 0) uniform CameraUBO {
  mat4 view;
  mat4 proj;
  vec3 eye;
  vec3 viewDir;
  float farDist;
  float nearDist;
} cam;

layout(binding = 1) SHADER_CONTROL_UNIFORM control;

layout(binding = 2) uniform sampler2D inGBuffPosition;
layout(binding = 3) uniform sampler2D inGBuffNormal;
layout(binding = 4) uniform sampler2D inGBuffColor;

layout(binding = 5) uniform sampler2D previousImage;

layout(binding = 6) uniform DynamicLightUBO {
  Light at[MAX_DYNAMIC_LOCAL_LIGHTS];
  uint count;
} dynLights;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 fragColor;

void main() {
  vec4 sampledPos = texture(inGBuffPosition, inUV);
  vec4 sampledNormal = texture(inGBuffNormal, inUV);
  vec4 sampledColor = texture(inGBuffColor, inUV);
  vec4 previousColor = texture(previousImage, inUV);
  
  float isObject    = sampledPos.w;
  float inMetallic  = sampledNormal.w;
  float inRoughness = sampledColor.w;
  vec3  inPos       = sampledPos.xyz;
  vec3  inColor     = sampledColor.xyz;
  
  if(int(isObject) == 1) {
    vec3 N = normalize(sampledNormal.xyz);
    vec3 V = normalize(cam.eye - inPos);
    
    //fragColor = vec4(N + vec3(1.0) / vec3(2.0), 1);
    //return;
    
    vec3 color = vec3(0, 0, 0);
    for(int i = 0; i < dynLights.count && control.doLocalLighting == 1; ++i) {
      vec3 lightColor = dynLights.at[i].color;
      vec3 lightPos   = dynLights.at[i].pos;
      vec3 lightAtten = dynLights.at[i].atten;
      float lightRad  = dynLights.at[i].radius;
      color += computeDirectPBR(lightColor, lightPos, lightAtten, lightRad,
                                inColor, inPos, N, V, inRoughness, inMetallic);
      //ComputeLighting(dynLights.at[i], inColor, inPos, N, V, inRoughness);
    }
    
    color += previousColor.xyz;
  
    fragColor = vec4(color.xyz, 1);
  }
  else {
    fragColor = previousColor;
  }
  
  // gamma correct as this is the final render pass
  fragColor.xyz *= control.toneMapExposure;
  fragColor.xyz = fragColor.xyz / (fragColor.xyz + vec3(1));
  fragColor.xyz = pow(fragColor.xyz, vec3(control.toneMapExponent / 2.2));
  //fragColor /= fragColor.a;
}