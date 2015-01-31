using System;
using System.Collections.Generic;
using MMF.Utility;
using SlimDX;
using SlimDX.Direct3D9;

namespace MMF.Model.Shape
{
    public class SilinderShape : ShapeBase
    {
        private readonly SilinderShapeDescription _desc;

        public SilinderShape(RenderContext context, Vector4 color,SilinderShapeDescription desc) : base(context, color)
        {
            _desc = desc;
        }

        public override string FileName
        {
            get { return "@@@SilinderShape@@@"; }
        }

        public override int VertexCount
        {
            get { return (int) (_desc.DivideCount*6*4); }
        }

        protected override void InitializeIndex(IndexBufferBuilder builder)
        {
            uint n = _desc.DivideCount + 1;
            for (uint i = 0; i < _desc.DivideCount; i++)
            {//ã‚ÌƒŠƒ“ƒO
                builder.AddSquare(i,i+1,(i+1)+n,i+n);
            }
            for (uint i = 0; i < _desc.DivideCount; i++)
            {//‰º‚ÌƒŠƒ“ƒO
                builder.AddSquare(i + n*2, i + n*3, (i + 1) + n*3, (i + 1) + n*2);
            }
            for (uint i = 0; i < _desc.DivideCount; i++)
            {//“à–Ê
                builder.AddSquare(i, i + n * 2, (i + 1) + n * 2, (i + 1));
            }
            for (uint i = 0; i < _desc.DivideCount; i++)
            {//“à–Ê
                builder.AddSquare(i+n, i+1 + n, (i + 1) + n * 3, i+n*3);
            }
        }

        protected override void InitializePositions(List<Vector4> positions)
        {
            appendRing(positions,1,1);
            appendRing(positions,1,_desc.Thickness+1f);
            appendRing(positions, -1, 1);
            appendRing(positions, -1, _desc.Thickness + 1f);
        }

        private void appendRing(List<Vector4> positions,float y,float r)
        {
            float stride = (float) (2*Math.PI/_desc.DivideCount);
            for (int i = 0; i <= _desc.DivideCount; i++)
            {
                positions.Add(new Vector4((float) (Math.Cos(i*stride)*r),y,(float) (Math.Sin(i*stride)*r),1f));
            }
        }

        public class SilinderShapeDescription
        {
            private float thickness;

            private uint divideCount;

            public SilinderShapeDescription(float thickness, uint divideCount)
            {
                this.thickness = thickness;
                this.divideCount = divideCount;
            }

            public float Thickness
            {
                get { return thickness; }
            }

            public uint DivideCount
            {
                get { return divideCount; }
            }
        }
    }
}