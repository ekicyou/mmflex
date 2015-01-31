using System.Runtime.InteropServices;
using SlimDX;

namespace MMF.MME.VariableSubscriber.ConstantSubscriber.ConstantBufferManager
{
    public struct FullMaterialConstantBufferInputLayout
    {
        public Vector4 AmbientColor;

        public Vector4 DiffuseColor;

        public Vector4 SpecularColor;

        public Vector4 EmissiveColor;

        public Vector4 ToonColor;

        public Vector4 EdgeColor;

        public Vector4 GroundShadowColor;

        public Vector4 AddingTexture;

        public Vector4  MultiplyingTexture;

        public Vector4 AddingSphereTexture;

        public Vector4 MultiplyingSphereTexture;

        public float SpecularPower;

        public static int SizeInBytes
        {
            get
            {
                int size = Marshal.SizeOf(typeof(FullMaterialConstantBufferInputLayout));
                size = size % 16 == 0 ? size : size + 16 - size % 16; //16の倍数じゃないとだめらしいので16の倍数にする
                return size;
            }
        }
    }
}