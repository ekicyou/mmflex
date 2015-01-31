using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMF.Model.Shape;
using SlimDX;

namespace MMF.Model.Controller.ControllerComponent
{
    class CenterCrossLine:IDrawable
    {
        private CubeShape xLine;

        private CubeShape yLine;

        private CubeShape zLine;

        private static float thickness = 0.1f;

        public CenterCrossLine(RenderContext context)
        {
            xLine=new CubeShape(context,new Vector4(1,0.55f,0,0.7f));
            yLine = new CubeShape(context, new Vector4(1, 0.55f, 0, 0.7f));
            zLine = new CubeShape(context, new Vector4(1, 0.55f, 0, 0.7f));
            xLine.Initialize();
            yLine.Initialize();
            zLine.Initialize();
            xLine.Transformer.Scale=new Vector3(30f,thickness,thickness);
            yLine.Transformer.Scale = new Vector3(thickness, 30f, thickness);
            zLine.Transformer.Scale = new Vector3(thickness, thickness, 30f);
        }

        public void AddTranslation(Vector3 trans)
        {
            xLine.Transformer.Position += trans;
            yLine.Transformer.Position += trans;
            zLine.Transformer.Position += trans;
        }
        

        public void Dispose()
        {
            xLine.Dispose();
            yLine.Dispose();
            zLine.Dispose();
        }

        public bool Visibility { get; set; }
        public string FileName { get; private set; }
        public int SubsetCount { get; private set; }
        public int VertexCount { get; private set; }
        public ITransformer Transformer { get; private set; }
        public void Draw()
        {
            xLine.Draw();
            yLine.Draw();
            zLine.Draw();
        }

        public void Update()
        {
        }

        public Vector4 SelfShadowColor { get; set; }
        public Vector4 GroundShadowColor { get; set; }
    }
}
