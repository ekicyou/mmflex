using System.Collections.Generic;
using MMF.Sprite;
using MMF.Utility;
using SlimDX;
using SlimDX.Direct3D11;

namespace MMF.Model.Shape
{
    public class PlaneBoard:IDrawable
    {

        public Buffer VertexBuffer;
        private EffectPass renderPass;
        private RenderContext context;
        private readonly ShaderResourceView _resView;
        private InputLayout VertexInputLayout;

        /// <summary>
        /// スプライトの描画に利用するエフェクト
        /// </summary>
        private Effect SpriteEffect { get; set; }

        public PlaneBoard(RenderContext context, ShaderResourceView resView):this(context,resView,new Vector2(200,200))
        {
            
        }
        public PlaneBoard(RenderContext context,ShaderResourceView resView,Vector2 size)
        {
            this.context = context;
            _resView = resView;
            Visibility = true;
            SpriteEffect = CGHelper.CreateEffectFx5FromResource("MMF.Resource.Shader.SpriteShader.fx", context.DeviceManager.Device);
            VertexInputLayout = new InputLayout(context.DeviceManager.Device, SpriteEffect.GetTechniqueByIndex(0).GetPassByIndex(0).Description.Signature, SpriteVertexLayout.InputElements);
            renderPass = SpriteEffect.GetTechniqueByIndex(0).GetPassByIndex(0);
            float width = size.X/2, height = size.Y/2;
            List<byte> vertexBytes=new List<byte>();
            CGHelper.AddListBuffer(new Vector3(-width, height, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector2(0, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector3(width, height, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector2(1, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector3(-width, -height, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector2(0, 1), vertexBytes);
            CGHelper.AddListBuffer(new Vector3(width, height, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector2(1, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector3(width, -height, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector2(1, 1), vertexBytes);
            CGHelper.AddListBuffer(new Vector3(-width, -height, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector2(0, 1), vertexBytes);
            using (DataStream ds = new DataStream(vertexBytes.ToArray(), true, true))
            {
                BufferDescription bufDesc = new BufferDescription()
                {
                    BindFlags = BindFlags.VertexBuffer,
                    SizeInBytes = (int)ds.Length
                };
                VertexBuffer = new SlimDX.Direct3D11.Buffer(context.DeviceManager.Device, ds, bufDesc);
            }
            Transformer=new BasicTransformer();
            Transformer.Scale=new Vector3(0.2f);
        }
        public void Dispose()
        {
            if(VertexBuffer!=null&&!VertexBuffer.Disposed)VertexBuffer.Dispose();
            if(VertexInputLayout!=null&&!VertexInputLayout.Disposed)VertexInputLayout.Dispose();
        }

        public bool Visibility { get; set; }
        public string FileName { get; private set; }
        public int SubsetCount { get; private set; }
        public int VertexCount { get; private set; }
        public ITransformer Transformer { get; private set; }
        public void Draw()
        {
            SpriteEffect.GetVariableBySemantic("WORLDVIEWPROJECTION")
                .AsMatrix()
                .SetMatrix(context.MatrixManager.makeWorldViewProjectionMatrix(this));
            SpriteEffect.GetVariableBySemantic("SPRITETEXTURE").AsResource().SetResource(_resView);
            context.DeviceManager.Context.InputAssembler.SetVertexBuffers(0, new VertexBufferBinding(VertexBuffer, SpriteVertexLayout.SizeInBytes, 0));
            context.DeviceManager.Context.InputAssembler.InputLayout = VertexInputLayout;
            context.DeviceManager.Context.InputAssembler.PrimitiveTopology = PrimitiveTopology.TriangleList;
            renderPass.Apply(context.DeviceManager.Context);
            context.DeviceManager.Context.Draw(12, 0);
        }

        public void Update()
        {
            
        }

        public Vector4 SelfShadowColor { get; set; }
        public Vector4 GroundShadowColor { get; set; }
    }
}
