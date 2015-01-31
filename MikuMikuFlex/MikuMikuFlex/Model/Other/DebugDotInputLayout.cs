﻿using System.Runtime.InteropServices;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;

namespace MMF.Model.Other
{
    public struct DebugDotInputLayout
    {
        public Vector3 Position;

        public static InputElement[] InputElements=new []{new InputElement()
        { 
            SemanticName = "POSITION",
            Format = Format.R32G32B32_Float
        }};

        public static int SizeInBytes
        {
            get
            {
                return Marshal.SizeOf(typeof (DebugDotInputLayout));
            }
        }
    }
}
