using System.Runtime.InteropServices;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;

namespace MMF.Sprite
{
    public struct SpriteVertexLayout
    {
        public Vector3 Position;

        public Vector2 UV;

        public static InputElement[] InputElements=new []{new InputElement()
        {
            SemanticName = "POSITION",
            Format = Format.R32G32B32_Float
        }, new InputElement()
        {
            SemanticName = "UV",
            Format = Format.R32G32_Float,
            AlignedByteOffset = InputElement.AppendAligned
        }, };

        public static int SizeInBytes
        {
            get
            {
                return Marshal.SizeOf(typeof (SpriteVertexLayout));
            }
        }
    }
}
