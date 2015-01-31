using System.Collections.Generic;
using MMF.Model;
using SlimDX;
using SlimDX.D3DCompiler;
using SlimDX.Direct3D11;

namespace MMF.Grid
{
    /// <summary>
    ///     デバッグ用にボーンの位置などを示す際などに利用するもの
    /// </summary>
    public class TargetCross : IDrawable
    {
        /// <summary>
        ///     軸の長さ
        /// </summary>
        private const int AxisLength = 3;

        /// <summary>
        ///     軸の入力レイアウト
        /// </summary>
        private InputLayout axisLayout;

        /// <summary>
        ///     軸の頂点数
        /// </summary>
        private int axisVectorCount;

        /// <summary>
        ///     軸の頂点バッファ
        /// </summary>
        private Buffer axisVertexBuffer;

        /// <summary>
        ///     エフェクト
        /// </summary>
        private Effect effect;

        /// <summary>
        ///     コンストラクタ
        /// </summary>
        public TargetCross()
        {
            Transformer = new BasicTransformer();
        }

        /// <summary>
        ///     可視性
        /// </summary>
        public bool IsVisibleAxisGrid { get; set; }

        /// <summary>
        ///     レンダリングに使用するコンテキスト
        /// </summary>
        private RenderContext RenderContext { get; set; }

        /// <summary>
        ///     描画
        /// </summary>
        public void Draw()
        {
            DeviceContext context = RenderContext.DeviceManager.Device.ImmediateContext;
            context.InputAssembler.PrimitiveTopology = PrimitiveTopology.LineList;
            effect.GetVariableBySemantic("WORLDVIEWPROJECTION")
                .AsMatrix()
                .SetMatrix(RenderContext.MatrixManager.makeWorldViewProjectionMatrix(Transformer.Scale,
                    Transformer.Rotation, Transformer.Position));
            if (IsVisibleAxisGrid)
            {
                context.InputAssembler.InputLayout = axisLayout;
                context.InputAssembler.SetVertexBuffers(0,
                    new VertexBufferBinding(axisVertexBuffer, AxisGridLayout.SizeInBytes, 0));
                effect.GetTechniqueByIndex(0).GetPassByIndex(1).Apply(context);
                context.Draw(7*axisVectorCount, 0);
            }
        }

        public int VertexCount { get; private set; }

        /// <summary>
        ///     変形に利用するITransformer
        /// </summary>
        public ITransformer Transformer { get; private set; }

        /// <summary>
        ///     Dispose
        /// </summary>
        public void Dispose()
        {
            
            axisVertexBuffer.Dispose();
            axisLayout.Dispose();
            effect.Dispose();
        }

        public void Update()
        {
        }

        /// <summary>
        ///     初期化
        /// </summary>
        /// <param name="renderContext">レンダリングに使用するコンテキスト</param>
        public void Load(RenderContext renderContext)
        {
            IsVisibleAxisGrid = true;
            RenderContext = renderContext;
            MakeGridVectors();
            SubsetCount = 1;
        }

        public void GetFileName()
        {

        }

        /// <summary>
        ///     バッファを作成する
        /// </summary>
        private void MakeGridVectors()
        {
            //エフェクトを読み込む
            using (ShaderBytecode byteCode = ShaderBytecode.CompileFromFile(@"Shader\grid.fx", "fx_5_0"))
            {
                effect = new Effect(RenderContext.DeviceManager.Device, byteCode);
            }
            //まずリストに頂点を格納
            List<float> axisVector = new List<float>();
            AddAxisVector(axisVector, AxisLength, 0, 0, new Vector4(1, 0, 0, 1));
            AddAxisVector(axisVector, 0, AxisLength, 0, new Vector4(0, 1, 0, 1));
            AddAxisVector(axisVector, 0, 0, AxisLength, new Vector4(0, 0, 1, 1));
            //バッファを作成
            using (DataStream vs = new DataStream(axisVector.ToArray(), true, true))
            {
                BufferDescription bufDesc = new BufferDescription
                {
                    BindFlags = BindFlags.VertexBuffer,
                    SizeInBytes = (int) vs.Length
                };
                axisVertexBuffer = new Buffer(RenderContext.DeviceManager.Device, vs, bufDesc);
            }
            axisVectorCount = axisVector.Count;
            //入力レイアウトを作成
            ShaderSignature v = effect.GetTechniqueByIndex(0).GetPassByIndex(1).Description.Signature;
            axisLayout = new InputLayout(RenderContext.DeviceManager.Device, v, AxisGridLayout.VertexElements);
            VertexCount = axisVectorCount;
        }

        /// <summary>
        ///     軸として頂点を格納する
        ///     x,y,zはどれか１つがnot 0,それ以外0を想定
        /// </summary>
        /// <param name="vertexList">頂点バッファ作成用リスト</param>
        /// <param name="x">xの長さ</param>
        /// <param name="y">yの長さ</param>
        /// <param name="z">zの長さ</param>
        /// <param name="color">色</param>
        private static void AddAxisVector(List<float> vertexList, float x, float y, float z, Vector4 color)
        {
            vertexList.Add(x);
            vertexList.Add(y);
            vertexList.Add(z);
            vertexList.Add(color.X);
            vertexList.Add(color.Y);
            vertexList.Add(color.Z);
            vertexList.Add(color.W);
            vertexList.Add(-x);
            vertexList.Add(-y);
            vertexList.Add(-z);
            vertexList.Add(color.X);
            vertexList.Add(color.Y);
            vertexList.Add(color.Z);
            vertexList.Add(color.W);
        }

        #region IDrawable メンバー

        public Vector4 SelfShadowColor { get; set; }

        public Vector4 GroundShadowColor { get; set; }

        public bool Visibility { get; set; }

        public string FileName { get; set; }
        public int SubsetCount { get; private set; }

        #endregion
    }
}