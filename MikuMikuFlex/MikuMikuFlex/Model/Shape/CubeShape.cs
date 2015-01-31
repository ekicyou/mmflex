
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMF.Utility;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;

namespace MMF.Model.Shape
{
    public class CubeShape: ShapeBase
    {
        public CubeShape(RenderContext context,Vector4 color) : base(context, color)
        {
        }

        public override string FileName
        {
            get { return "@@@CubeShape@@@"; }
        }

        public override int VertexCount
        {
            get { return 36; }
        }

        protected override void InitializeIndex(IndexBufferBuilder builder)
        {
            builder.AddSquare(0, 1, 2, 3);
            builder.AddSquare(6, 5, 4, 7);
            builder.AddSquare(0, 3, 7, 4);
            builder.AddSquare(2, 1, 5, 6);
            builder.AddSquare(3, 2, 6, 7);
            builder.AddSquare(1, 0, 4, 5);
        }

        protected override void InitializePositions(List<Vector4> positions)
        {
            
                positions.Add(new Vector4(-1, 1, -1, 1));
                positions.Add(new Vector4(-1, 1, 1, 1));
                positions.Add(new Vector4(1, 1, 1, 1));
                positions.Add(new Vector4(1, 1, -1, 1));
                positions.Add(new Vector4(-1, -1, -1, 1));
                positions.Add(new Vector4(-1, -1, 1, 1));
                positions.Add(new Vector4(1, -1, 1, 1));
                positions.Add(new Vector4(1, -1, -1, 1));
            
        }
    }
}
