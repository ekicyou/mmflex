﻿using System;
using System.Runtime.InteropServices;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;

namespace MMF.Model
{
    /// <summary>
    ///     MMDModel用のInputLayout
    /// </summary>
    public struct BasicInputLayout
    {
        public static readonly InputElement[] VertexElements =
        {
            new InputElement
            {
                SemanticName = "POSITION",
                Format = Format.R32G32B32A32_Float
            },new InputElement
            {
                SemanticName = "BLENDWEIGHT",
                Format = Format.R32G32B32A32_Float,
                AlignedByteOffset = InputElement.AppendAligned
            },
            new InputElement
            {
                SemanticName = "BLENDINDICES",
                Format = Format.R32G32B32A32_UInt,
                AlignedByteOffset = InputElement.AppendAligned
            }
            , 
            new InputElement
            {
                SemanticName = "NORMAL",
                Format = Format.R32G32B32_Float,
                AlignedByteOffset = InputElement.AppendAligned
            }, 
            new InputElement//Tex
            {
                SemanticName = "TEXCOORD",
                Format = Format.R32G32_Float,
                SemanticIndex = 0,
                AlignedByteOffset = InputElement.AppendAligned
            },
            new InputElement//AddUV1
            {
                SemanticName = "TEXCOORD",
                SemanticIndex = 1,
                Format = Format.R32G32B32A32_Float,
                AlignedByteOffset = InputElement.AppendAligned
            },new InputElement//AddUV2
            {
                SemanticName = "TEXCOORD",
                SemanticIndex = 2,
                Format = Format.R32G32B32A32_Float,
                AlignedByteOffset = InputElement.AppendAligned
            },new InputElement//AddUV3
            {
                SemanticName = "TEXCOORD",
                SemanticIndex = 3,
                Format = Format.R32G32B32A32_Float,
                AlignedByteOffset = InputElement.AppendAligned
            },
             new InputElement//AddUV4
            {
                SemanticName = "TEXCOORD",
                SemanticIndex = 4,
                Format = Format.R32G32B32A32_Float,
                AlignedByteOffset = InputElement.AppendAligned
            }, 
             new InputElement//SDEF-C
            {
                SemanticName = "TEXCOORD",
                SemanticIndex = 5,
                Format = Format.R32G32B32A32_Float,
                AlignedByteOffset = InputElement.AppendAligned
            },
            new InputElement//SDEF-R0
            {
                SemanticName = "TEXCOORD",
                SemanticIndex = 6,
                Format = Format.R32G32B32_Float,
                AlignedByteOffset = InputElement.AppendAligned
            },
            new InputElement//SDEF-R1
            {
                SemanticName = "TEXCOORD",
                SemanticIndex = 7,
                Format = Format.R32G32B32_Float,
                AlignedByteOffset = InputElement.AppendAligned
            },
            new InputElement
            {
                SemanticName = "TEXCOORD",
                Format = Format.R32_Float,
                SemanticIndex = 8,
                AlignedByteOffset = InputElement.AppendAligned
            }, 
            new InputElement
            {
                SemanticName = "PSIZE",
                SemanticIndex = 15,
                Format = Format.R32_UInt,
                AlignedByteOffset = InputElement.AppendAligned
            }, 
        };

        #region 順番入れ替え危険

        public Vector4 Position;

        public float BoneWeight1;

        public float BoneWeight2;

        public float BoneWeight3;

        public float BoneWeight4;

        public UInt32 BoneIndex1;

        public UInt32 BoneIndex2;

        public UInt32 BoneIndex3;

        public UInt32 BoneIndex4;

        public Vector3 Normal;

        public Vector2 UV;

        public Vector4 AddUV1;

        public Vector4 AddUV2;

        public Vector4 AddUV3;

        public Vector4 AddUV4;

        public Vector4 Sdef_C;

        public Vector3 SdefR0;

        public Vector3 SdefR1;

        public float EdgeWeight;

        public UInt32 Index;


        #endregion

        public static int SizeInBytes
        {
            get { return Marshal.SizeOf(typeof (BasicInputLayout)); }
        }
    }
}