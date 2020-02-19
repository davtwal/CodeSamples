// Materials:
// Albedo = 0
// Normal = 1
// Metallic = 2
// Roughness = 3
// AO/Sheen = 4

#define MAX_MATERIALS 4
#define MTL_MAP_COUNT 4

#define MAX_GLOBAL_LIGHTS 2
#define MAX_DYNAMIC_LOCAL_LIGHTS 128

#define SHADER_CONTROL_UNIFORM  \
  uniform ShaderControl {       \
    float momentBias;           \
    float depthBias;            \
    float defaultRoughness;     \
    float defaultMetallic;      \
    float toneMapExposure;      \
    float toneMapExponent;      \
    int   doLocalLighting;      \
    int   doGlobalLighting;     \
    int   doShadows;            \
    int   doIBLLighting;        \
    int   enableHDRBackground;  \
  }