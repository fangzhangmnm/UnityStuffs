// credit https://roystan.net/articles/toon-water.html
Shader "Custom/ToonWater"
{
    Properties
    {	
        _DepthGradientShallow("Depth Gradient Shallow", Color) = (0.325, 0.807, 0.971, 0.725)
        _DepthGradientDeep("Depth Gradient Deep", Color) = (0.086, 0.407, 1, 0.749)
        _DepthMaxDistance("Depth Maximum Distance", Float) = 1
        _SurfaceNoise("Surface Noise", 2D) = "white" {}
        _SurfaceNoiseCutoff("Surface Noise Cutoff", Range(0, 1)) = 0.777
        _FoamMaxDistance("Foam Maximum Distance", Float) = 0.4
        _FoamMinDistance("Foam Minimum Distance", Float) = 0.04
        _SurfaceNoiseScroll("Surface Noise Scroll Amount", Vector) = (0.03, 0.03, 0, 0)
        _SurfaceDistortion("Surface Distortion", 2D) = "white" {}	
        _SurfaceDistortionAmount("Surface Distortion Amount", Range(0, 1)) = 0.27
        _FoamColor("Foam Color", Color) = (1,1,1,1)
    }
    SubShader
    {
		Tags
		{
			"Queue" = "Transparent"
		}
        Pass
        {
            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
			CGPROGRAM
            #define SMOOTHSTEP_AA 0.01
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float4 uv : TEXCOORD0;
                float3 normal : NORMAL;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                float4 screenPosition : TEXCOORD2;
                float2 noiseUV : TEXCOORD0;
                float2 distortUV : TEXCOORD1;
                float3 viewNormal : NORMAL;
            };
            sampler2D _SurfaceNoise;
            float4 _SurfaceNoise_ST;
            sampler2D _SurfaceDistortion;
            float4 _SurfaceDistortion_ST;
            v2f vert (appdata v)
            {
                v2f o;

                o.vertex = UnityObjectToClipPos(v.vertex);
                o.screenPosition = ComputeScreenPos(o.vertex);
                o.noiseUV = TRANSFORM_TEX(v.uv, _SurfaceNoise);
                o.distortUV = TRANSFORM_TEX(v.uv, _SurfaceDistortion);
                o.viewNormal = COMPUTE_VIEW_NORMAL;

                return o;
            }
            float4 alphaBlend(float4 top, float4 bottom)
            {
                float3 color = (top.rgb * top.a) + (bottom.rgb * (1 - top.a));
                float alpha = top.a + bottom.a * (1 - top.a);

                return float4(color, alpha);
            }
            float4 _DepthGradientShallow;
            float4 _DepthGradientDeep;
            float _DepthMaxDistance;
            float _SurfaceNoiseCutoff;
            float _FoamMaxDistance;
            float _FoamMinDistance;
            float2 _SurfaceNoiseScroll;
            sampler2D _CameraDepthTexture;
            float _SurfaceDistortionAmount;
            sampler2D _CameraDepthNormalsTexture;

            float4 _FoamColor;

			float3 getCamPos(sampler2D _CameraDepthTexture, float2 screenPosition) {
				float depth10 = tex2D(_CameraDepthTexture, screenPosition);
				float4 ndc = float4(2 * screenPosition - 1, 1 - 2 * depth10, 1);
				float4 camPos = mul(unity_CameraInvProjection, ndc);
				return camPos.xyz / camPos.w;
			}
			float3 getCamNormal(sampler2D _CameraDepthTexture, float2 screenPosition) {
				float3 p0 = getCamPos(_CameraDepthTexture, screenPosition);
				float3 p1 = getCamPos(_CameraDepthTexture, float2(screenPosition.x+.002, screenPosition.y));
				float3 p2 = getCamPos(_CameraDepthTexture, float2(screenPosition.x, screenPosition.y+.002));
				return normalize(cross(p1 - p0, p2 - p0));
				//float3 p3 = getCamPos(_CameraDepthTexture, float2(screenPosition.x - .002, screenPosition.y));
				//float3 p4 = getCamPos(_CameraDepthTexture, float2(screenPosition.x, screenPosition.y - .002));
				//return normalize(cross(p1 - p0, p2 - p0)+cross(p3-p0,p4-p0));
			}
            float4 frag (v2f i) : SV_Target
            { 
				float2 screenPosition = i.screenPosition.xy / i.screenPosition.w;
				float existingDepthLinear = LinearEyeDepth(tex2D(_CameraDepthTexture, screenPosition));
				float3 existingNormal = getCamNormal(_CameraDepthTexture, screenPosition);
                float depthDifference = existingDepthLinear - i.screenPosition.w;
                float waterDepthDifference01 = saturate(depthDifference / _DepthMaxDistance);
                float4 waterColor = lerp(_DepthGradientShallow, _DepthGradientDeep, waterDepthDifference01);


                float3 normalDot = saturate(dot(existingNormal, i.viewNormal));

                float foamDistance = lerp(_FoamMaxDistance, _FoamMinDistance, normalDot);
                float foamDepthDifference01 = saturate(depthDifference / foamDistance);

                float surfaceNoiseCutoff = foamDepthDifference01 * _SurfaceNoiseCutoff;
                float2 distortSample = (tex2D(_SurfaceDistortion, i.distortUV).xy * 2 - 1) * _SurfaceDistortionAmount;
                float2 noiseUV = float2((i.noiseUV.x + _Time.y * _SurfaceNoiseScroll.x) + distortSample.x, (i.noiseUV.y + _Time.y * _SurfaceNoiseScroll.y) + distortSample.y);
                float surfaceNoiseSample = tex2D(_SurfaceNoise, noiseUV).r;
                float surfaceNoise = smoothstep(surfaceNoiseCutoff - SMOOTHSTEP_AA, surfaceNoiseCutoff + SMOOTHSTEP_AA, surfaceNoiseSample);
                float4 surfaceNoiseColor = _FoamColor;
                surfaceNoiseColor.a *= surfaceNoise;
                return alphaBlend(surfaceNoiseColor, waterColor);
            }
            ENDCG
        }
    }
}