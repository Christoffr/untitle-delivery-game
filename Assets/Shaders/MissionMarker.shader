Shader "Custom/MissionMarker"
{
    Properties
    {
        _Color ("Color", Color) = (1,0,0,1)
        _Intensity ("Intensity", Float) = 2
        _HeightFade ("Height Fade", Float) = 2
        _EdgeSoftness ("Edge Softness", Float) = 3
        _PulseSpeed ("Pulse Speed", Float) = 2
    }

    SubShader
    {
        Tags { "RenderType"="Transparent" "Queue"="Transparent" }
        Blend One One // Additive
        ZWrite Off
        Cull Off

        Pass
        {
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normal : NORMAL;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 positionOS : TEXCOORD0;
            };

            float4 _Color;
            float _Intensity;
            float _HeightFade;
            float _EdgeSoftness;
            float _PulseSpeed;

            Varyings vert (Attributes v)
            {
                Varyings o;
                o.positionCS = TransformObjectToHClip(v.positionOS);
                o.positionOS = v.positionOS;
                return o;
            }

            float4 frag (Varyings i) : SV_Target
            {
                float3 pos = i.positionOS;

                // Normalize local space (assuming pivot center)
                float2 xz = pos.xz;

                // Horizontal falloff (soft edges)
                float dist = length(xz);
                float edgeFade = saturate(1 - pow(dist, _EdgeSoftness));

                // Vertical fade
                float heightFade = saturate(1 - pos.y * _HeightFade);

                // Combine fades
                float alpha = edgeFade * heightFade;

                float3 color = _Color.rgb * alpha * _Intensity;

                return float4(color, alpha);
            }
            ENDHLSL
        }
    }
}