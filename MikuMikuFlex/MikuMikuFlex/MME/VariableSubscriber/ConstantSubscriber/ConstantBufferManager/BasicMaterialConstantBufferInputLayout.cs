using System.Runtime.InteropServices;
using SlimDX;

namespace MMF.MME.VariableSubscriber.ConstantSubscriber.ConstantBufferManager
{
    public struct BasicMaterialConstantBufferInputLayout
    {
        #region 順番入れ替え危険
        public Vector4 AmbientLight;

        public Vector4 DiffuseLight;

        public Vector4 SpecularLight;

        public float SpecularPower;
        #endregion
        public static int SizeInBytes
        {
            get
            {
                int size = Marshal.SizeOf(typeof (BasicMaterialConstantBufferInputLayout));
                size = size%16 == 0 ? size : size + 16 - size%16; //16の倍数じゃないとだめらしいので16の倍数にする
                return size;
            }
        }
    }
}