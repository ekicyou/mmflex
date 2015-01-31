using System.Collections.Generic;
using System.Drawing;
using MMF.DeviceManager;
using MMF.Utility;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;

namespace MMF.Model.Shape
{
    public abstract class ShapeBase : IDrawable,IHitTestable
    {
        protected Vector4 _color;
        private InputLayout layout;
        private Effect effect;
        private Buffer indexBuffer;
        private Buffer vertexBuffer;

        public ShapeBase(RenderContext context,Vector4 color)
        {
            _color = color;
            this.RenderContext = context;
            Visibility = true;
            SubsetCount = 1;
            Transformer=new BasicTransformer();
            RenderContext = context;
        }

        public RenderContext RenderContext { get; set; }
        public bool Visibility { get; set; }
        public abstract string FileName { get; }
        public int SubsetCount { get; private set; }
        public abstract int VertexCount { get; }
        public ITransformer Transformer { get; private set; }
        public Vector4 SelfShadowColor { get; set; }
        public Vector4 GroundShadowColor { get; set; }

        public void Dispose()
        {
            if(indexBuffer!=null&&!indexBuffer.Disposed)indexBuffer.Dispose();
            if (vertexBuffer != null && !vertexBuffer.Disposed) vertexBuffer.Dispose();
            if (effect != null && !effect.Disposed) effect.Dispose();
            if(layout!=null&&!layout.Disposed)layout.Dispose();
        }

        public void Initialize()
        {
            effect = CGHelper.CreateEffectFx5FromResource(@"MMF.Resource.Shader.ShapeShader.fx",
                RenderContext.DeviceManager.Device);
            List<Vector4> positions=new List<Vector4>();
            InitializePositions(positions);
            vertexBuffer = CGHelper.CreateBuffer(positions, RenderContext.DeviceManager.Device, BindFlags.VertexBuffer);
            IndexBufferBuilder builder=new IndexBufferBuilder(RenderContext);
            InitializeIndex(builder);
            indexBuffer = builder.build();
            layout = new InputLayout(RenderContext.DeviceManager.Device,
                effect.GetTechniqueByIndex(0).GetPassByIndex(0).Description.Signature, ShapeInputLayout.VertexElements);
        }

        protected abstract void InitializeIndex(IndexBufferBuilder builder);

        protected abstract void InitializePositions(List<Vector4> positions);


        public void Draw()
        {
            DeviceContext context = RenderContext.DeviceManager.Device.ImmediateContext;
            context.InputAssembler.PrimitiveTopology=PrimitiveTopology.TriangleList;
            effect.GetVariableBySemantic("COLOR").AsVector().Set(_color);
            effect.GetVariableBySemantic("WORLDVIEWPROJECTION")
                .AsMatrix()
                .SetMatrix(RenderContext.MatrixManager.makeWorldViewProjectionMatrix(this));
            context.InputAssembler.InputLayout = layout;
            context.InputAssembler.SetIndexBuffer(indexBuffer,Format.R32_UInt,0);
            context.InputAssembler.SetVertexBuffers(0,new VertexBufferBinding(vertexBuffer,ShapeInputLayout.SizeInBytes,0));
            effect.GetTechniqueByIndex(0).GetPassByIndex(0).Apply(context);
            context.DrawIndexed(VertexCount,0,0);
        }

        public void Update()
        {
            
        }

        public void RenderHitTestBuffer(float col)
        {
            DeviceContext context = RenderContext.DeviceManager.Device.ImmediateContext;
            context.InputAssembler.PrimitiveTopology = PrimitiveTopology.TriangleList;
            effect.GetVariableBySemantic("COLOR").AsVector().Set(new Vector4(col,0,0,0));
            effect.GetVariableBySemantic("WORLDVIEWPROJECTION")
                .AsMatrix()
                .SetMatrix(RenderContext.MatrixManager.makeWorldViewProjectionMatrix(this));
            context.InputAssembler.InputLayout = layout;
            context.InputAssembler.SetIndexBuffer(indexBuffer, Format.R32_UInt, 0);
            context.InputAssembler.SetVertexBuffers(0, new VertexBufferBinding(vertexBuffer, ShapeInputLayout.SizeInBytes, 0));
            effect.GetTechniqueByIndex(0).GetPassByIndex(1).Apply(context);
            context.DrawIndexed(VertexCount, 0, 0);
        }

        public virtual void HitTestResult(bool result, bool mouseState, Point mousePosition)
        {
            
        }
    }
}