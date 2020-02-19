// Copyright (C) 2019 DigiPen Institute of Technology
// FXAA Implementation

#version 450

#define SIZE_X 8
#define SIZE_Y 8

layout(constant_id = 0) const float LUMINANCE_FRACTION      = .587 / .299;  // How much influence the red channel has in luminance
layout(constant_id = 1) const float FXAA_EDGE_THRESHOLD_MIN = 1.0 / 16.0;   // The minimum threshold for edge detection
layout(constant_id = 2) const float FXAA_EDGE_THRESHOLD     = 1.0 / 4.0;    // The amount of luminance difference relative to the maximum luminance that is required to determine an edge
layout(constant_id = 3) const float FXAA_SUBPIX_TRIM        = 1.0 / 3.0;    // 
layout(constant_id = 4) const float FXAA_SUBPIX_TRIM_SCALE  = 1.0 / 1.0;    // 
layout(constant_id = 5) const float FXAA_SUBPIX_CAP         = 3.0 / 4.0;    // The farthest the subpixel will move when sampling
layout(constant_id = 6) const uint  FXAA_SEARCH_STEPS       = 12;           // The farthest away the algorithm will search for the end of edge
layout(constant_id = 7) const float FXAA_SEARCH_THRESHOLD   = 1.0 / 4.0;    // The threshold that must be met for the algorithm to determine end of edge

// the frame itself
layout(binding = 0) uniform sampler2D image;

layout(location = 0) in vec2 inUV;

layout(location = 0) out vec4 fragColor;

// Converts an RGB value into a scalar luminance value, estimating how bright it is.
float fxaaLuma(vec3 rgb) {
    return rgb.y * LUMINANCE_FRACTION + rgb.x;
}

void main() {
    // FXAA STEP 0: IMAGE LOADING
    ivec2 size  = textureSize(image, 0);

    // du_dv: the uv distance between each pixel
    vec2  du_dv = vec2(1.0 / (size.x - 1), 1.0 / (size.y - 1));

    vec2 centerLoad = inUV;
    vec2 northLoad      = vec2(inUV.x, inUV.y - du_dv.y);
    vec2 southLoad      = vec2(inUV.x, inUV.y + du_dv.y);
    vec2 westLoad       = vec2(inUV.x - du_dv.y, inUV.y);
    vec2 eastLoad       = vec2(inUV.x + du_dv.x, inUV.y);
    vec2 northWestLoad  = vec2(westLoad.x, northLoad.y);
    vec2 northEastLoad  = vec2(eastLoad.x, northLoad.y);
    vec2 southWestLoad  = vec2(westLoad.x, southLoad.y);
    vec2 southEastLoad  = vec2(eastLoad.x, southLoad.y);
    
    vec3 rgbN = texture(image, northLoad).xyz;
    vec3 rgbS = texture(image, southLoad).xyz;
    vec3 rgbE = texture(image, eastLoad).xyz;
    vec3 rgbW = texture(image, westLoad).xyz;
    vec3 rgbC = texture(image, centerLoad).xyz;

    // FXAA STEP 1: LUMINANCE CONVERSION & LOCAL CONTRAST CHECK
    float lumaN = fxaaLuma(rgbN);
    float lumaS = fxaaLuma(rgbS);
    float lumaE = fxaaLuma(rgbE);
    float lumaW = fxaaLuma(rgbW);
    float lumaC = fxaaLuma(rgbC); // this algorithm is lumaC! (lunacy)

    // Determine the range of lumance within the cross
    float rangeMin = min(lumaN, min(lumaS, min(lumaE, min(lumaW, lumaC))));
    float rangeMax = max(lumaN, max(lumaS, max(lumaE, max(lumaW, lumaC))));
    float range = rangeMax - rangeMin;

    if(range > max(FXAA_EDGE_THRESHOLD_MIN, rangeMax * FXAA_EDGE_THRESHOLD)) {
        // FXAA STEP 2: SUB-PIXEL ALIASING TEST
        float lumaL = (lumaN + lumaS + lumaE + lumaW) * .25;
        float rangeL = abs(lumaL - lumaC);
        float blendL = min(max(0.0, (rangeL / range - FXAA_SUBPIX_TRIM) * FXAA_SUBPIX_TRIM_SCALE), FXAA_SUBPIX_CAP);
        
        // FXAA STEP 3: CLASSIFY PIXELS AS HORIZONTAL/VERTICAL EDGES
        vec3 rgbNW = texture(image, northWestLoad).xyz;//loadedData[NW.x][NW.y].xyz;
        vec3 rgbNE = texture(image, northEastLoad).xyz;//loadedData[NE.x][NE.y].xyz;
        vec3 rgbSW = texture(image, southWestLoad).xyz;//loadedData[SW.x][SW.y].xyz;
        vec3 rgbSE = texture(image, southEastLoad).xyz;//loadedData[SE.x][SE.y].xyz;
        float lumaNW = fxaaLuma(rgbNW);
        float lumaNE = fxaaLuma(rgbNE);
        float lumaSW = fxaaLuma(rgbSW);
        float lumaSE = fxaaLuma(rgbSE);

        float edgeVert = 
            abs((.25 * lumaNW) + (-.5 * lumaN) + (.25 * lumaNE)) +
            abs((.50 * lumaW ) + (-1  * lumaC) + (.50 * lumaE )) +
            abs((.25 * lumaSW) + (-.5 * lumaS) + (.25 * lumaSE));

            
        float edgeHorz = 
            abs((.25 * lumaNW) + (-.5 * lumaW) + (.25 * lumaSW)) +
            abs((.50 * lumaN ) + (-1  * lumaC) + (.50 * lumaS )) +
            abs((.25 * lumaNE) + (-.5 * lumaE) + (.25 * lumaSE));

        // is local edge direction horizontal?
        bool horzSpan = edgeHorz >= edgeVert;

        // FXAA STEP 4: SELECT HIGHEST CONTRAST PIXEL PAIR 90 DEGREES TO EGDE
        bool doneN = false;
        bool doneP = false;

        vec2 offNP;
        float gradientN;
        float gradientP;
        if(horzSpan) {
            offNP = vec2(du_dv.x, 0);
            gradientN = lumaN - lumaC;
            gradientP = lumaS - lumaC;
        }
        else {
            offNP = vec2(0, du_dv.y);
            gradientN = lumaW - lumaC;
            gradientP = lumaE - lumaC;
        }

        bool pairIsN = abs(gradientN) > abs(gradientP);
        float gradient = max(abs(gradientN), abs(gradientP));
        
        vec2 posN = centerLoad - offNP;
        vec2 posP = centerLoad + offNP;
        float lumaEndN = fxaaLuma(texture(image, posN).xyz);
        float lumaEndP = fxaaLuma(texture(image, posP).xyz);
        float lumaRem = pairIsN ? lumaEndN : lumaEndP;

        // always not true here - TODO
        bool lumaMLTZero = lumaC - lumaRem < 0.0;

        // FXAA STEP 5: SEARCH FOR END-OF-EDGE IN DIRECTIONS
        for(uint i = 0; i < FXAA_SEARCH_STEPS; ++i) {
            if(!doneN)
                lumaEndN = fxaaLuma(textureGrad(image, posN, offNP, offNP).xyz);
            if(!doneP)
                lumaEndP = fxaaLuma(textureGrad(image, posP, offNP, offNP).xyz);
            
            doneN = doneN || abs(lumaEndN - lumaRem) >= gradient;
            doneP = doneP || abs(lumaEndP - lumaRem) >= gradient;
            if(doneN && doneP) break;
            if(!doneN) posN -= offNP;
            if(!doneP) posP += offNP;
        }
        
        // FXAA STEP 6: TRANSFORM INTO SUB-PIXEL SHIFT
        float dstN, dstP;
        if(horzSpan) {
            dstN = centerLoad.x - posN.x;
            dstP = posP.x - centerLoad.x;
        } else {
            dstN = centerLoad.y - posN.y;
            dstP = posP.y - centerLoad.y;
        }

        float spanLen = dstN + dstP;
        float spanLenRcp = 1.0 / spanLen;
        float dst = min(dstN, dstP);

        bool dirN = dstN < dstP;
        bool goodSpanN = lumaEndN < 0 != lumaMLTZero;
        bool goodSpanP = lumaEndP < 0 != lumaMLTZero;
        bool goodSpan = dirN ? goodSpanN : goodSpanP;

        // This is some sort of magic that is not explained in the white paper
        // I need to go back over this, though I do have some idea of what it's doing
        float C = clamp(abs(((lumaS + lumaN + lumaW + lumaE) * 2 + (lumaNW + lumaSW + lumaNE + lumaSE)) * 1/12 - lumaC) / range, 0, 1);
        float subpixIntermediate = (-2 * C + 3) * (C * C);
        float subpix = .75 * (subpixIntermediate * subpixIntermediate);

        float pixelOffset = dst * -spanLenRcp + .5;
        float pixelOffsetSubpix = max(goodSpan ? pixelOffset : 0, subpix);

        // FXAA STEP 7: RE-SAMPLE GIVEN SUB-PIXEL OFFSET
        if(horzSpan) {
            centerLoad.x += pixelOffsetSubpix * du_dv.x * (pairIsN ? -1 : 1);
        } else {
            centerLoad.y += pixelOffsetSubpix * du_dv.y * (pairIsN ? -1 : 1);
        }

        // FXAA STEP 8: Blend in a low-pass filter depending on the amount of luma.
        vec3 rgbL  = (rgbN + rgbS + rgbE + rgbW + rgbC + rgbNW + rgbNE + rgbSW + rgbSE) 
                        * vec3(1.0 / 9.0);

        fragColor = vec4(rgbL * blendL + (1 - blendL) * texture(image, centerLoad).xyz, 1);
    }
    else {
        // no AA needed
        fragColor = vec4(rgbC, 1);
    }
}