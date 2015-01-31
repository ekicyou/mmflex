using System.Collections.Generic;
using MMF.Utility;
using SlimDX;

namespace MMF.Model.Shape
{
    public class ConeShape : ShapeBase
    {
        public ConeShape(RenderContext context, Vector4 color) : base(context, color)
        {
        }

        public override string FileName
        {
            get { return "@@@ConeShape@@@"; }
        }

        public override int VertexCount
        {
            get { return 18; }
        }

        protected override void InitializeIndex(IndexBufferBuilder builder)
        {
            builder.AddSquare(4,3,2,1);
            builder.AddTriangle(0,3,4);
            builder.AddTriangle(0,2,3);
            builder.AddTriangle(0,1,2);
            builder.AddTriangle(0,4,1);
        }

        protected override void InitializePositions(List<Vector4> positions)
        {
            positions.Add(new Vector4(0, 1, 0, 1));
            positions.Add(new Vector4(-1, -1, 0, 1));
            positions.Add(new Vector4(0, -1, 1, 1));
            positions.Add(new Vector4(1, -1, 0, 1));
            positions.Add(new Vector4(0, -1, -1, 1));
        }
    }
}