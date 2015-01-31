using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;

namespace MMF.Model.Shape
{
    public struct ShapeInputLayout
    {
        public static readonly InputElement[] VertexElements =
        {
            new InputElement
            {
                SemanticName="POSITION",
                Format = Format.R32G32B32A32_Float
            }
        };

        public Vector4 Position;

        public static int SizeInBytes
        {
            get { return Marshal.SizeOf(typeof (ShapeInputLayout)); }
        }
    }
}
