using System.Collections.Generic;
using MMF.Model;
using MMF.Utility;
using SlimDX;
using SlimDX.D3DCompiler;
using SlimDX.Direct3D11;

namespace MMF.Grid
{
    /// <summary>
    ///     標準的なグリッド
    /// </summary>
    public class BasicGrid : IDrawable
    {
        /// <summary>
        ///     グリッドの長さ
        /// </summary>
        private const int GridLength = 100;

        /// <summary>
        ///     グリッドの幅
        /// </summary>
        private const int GridWidth = 10;

        /// <summary>
        ///     グリッドの幅
        /// </summary>
        private const int GridCount = 20;

        /// <summary>
        ///     軸の長さ
        /// </summary>
        private const int AxisLength = 300;

        /// <summary>
        ///     軸の入力レイアウト
        /// </summary>
        private InputLayout axisLayout;

        /// <summary>
        ///     軸のカウント
        /// </summary>
        private int axisVectorCount;

        /// <summary>
        ///     軸の頂点バッファ
        /// </summary>
        private Buffer axisVertexBuffer;

        /// <summary>
        ///     レンダリングに利用するエフェクト
        /// </summary>
        private Effect effect;

        /// <summary>
        ///     グリッドの入力レイアウト
        /// </summary>
        private InputLayout layout;

        /// <summary>
        ///     グリッドのカウント
        /// </summary>
        private int vectorCount;

        /// <summary>
        ///     グリッドの頂点バッファ
        /// </summary>
        private Buffer vertexBuffer;

        private bool _isVisibleMeasureGrid;
        private bool _isVisibleAxisGrid;

        /// <summary>
        ///     コンストラクタ
        /// </summary>
        public BasicGrid()
        {
            Transformer = new BasicTransformer();
            Visibility = true;
            FileName = "@MMF.CG.Model.Grid.BasicGrid@";
        }

        /// <summary>
        ///     グリッドが見えるかどうか
        /// </summary>
        public bool IsVisibleMeasureGrid
        {
            get { return _isVisibleMeasureGrid; }
            set { _isVisibleMeasureGrid = value; }
        }

        /// <summary>
        ///     軸が見えるかどうか
        /// </summary>
        public bool IsVisibleAxisGrid
        {
            get { return _isVisibleAxisGrid; }
            set { _isVisibleAxisGrid = value; }
        }

        /// <summary>
        ///     レンダリングに利用するコンテキスト
        /// </summary>
        private RenderContext RenderContext { get; set; }

        public void Update()
        {
        }

        /// <summary>
        ///     描画
        /// </summary>
        public void Draw()
        {
            DeviceContext context = RenderContext.DeviceManager.Device.ImmediateContext;
            context.InputAssembler.PrimitiveTopology = PrimitiveTopology.LineList;
            effect.GetVariableBySemantic("WORLDVIEWPROJECTION")
                .AsMatrix()
                .SetMatrix(RenderContext.MatrixManager.makeWorldViewProjectionMatrix(this));
            if (IsVisibleMeasureGrid)
            {
                context.InputAssembler.InputLayout = layout;
                context.InputAssembler.SetVertexBuffers(0,
                    new VertexBufferBinding(vertexBuffer, MeasureGridLayout.SizeInBytes, 0));
                effect.GetTechniqueByIndex(0).GetPassByIndex(0).Apply(context);
                context.Draw(4*vectorCount, 0);
            }
            if (IsVisibleAxisGrid)
            {
                context.InputAssembler.InputLayout = axisLayout;
                context.InputAssembler.SetVertexBuffers(0,
                    new VertexBufferBinding(axisVertexBuffer, AxisGridLayout.SizeInBytes, 0));
                effect.GetTechniqueByIndex(0).GetPassByIndex(1).Apply(context);
                context.Draw(7*axisVectorCount, 0);
            }
        }

        public void GetFileName()
        {

        }

        public int VertexCount { get; private set; }

        /// <summary>
        ///     移動に利用するITransformer
        /// </summary>
        public ITransformer Transformer { get; private set; }

        /// <summary>
        ///     IDisposable実装
        /// </summary>
        public void Dispose()
        {
            axisVertexBuffer.Dispose();
            vertexBuffer.Dispose();
            layout.Dispose();
            axisLayout.Dispose();
            effect.Dispose();
        }

        /// <summary>
        ///     初期化
        /// </summary>
        /// <param name="renderContext">びゅがに利用するレンダリングコンテキスト</param>
        public void Load(RenderContext renderContext)
        {
            IsVisibleAxisGrid = true;
            IsVisibleMeasureGrid = true;
            RenderContext = renderContext;
            MakeGridVectors();
            SubsetCount = 1;
        }

        /// <summary>
        ///     頂点バッファ作成
        /// </summary>
        private void MakeGridVectors()
        {
            List<Vector3> gridVector = new List<Vector3>();
            effect = CGHelper.CreateEffectFx5FromResource(@"MMF.Resource.Shader.GridShader.fx",
                RenderContext.DeviceManager.Device);
            for (int i = 0; i <= GridCount; i++)
            {
                if (i != GridCount/2)
                {
                    gridVector.Add(new Vector3(-GridLength, 0, -GridLength + i*GridWidth));
                    gridVector.Add(new Vector3(GridLength, 0, -GridLength + i*GridWidth));
                }
            }
            for (int i = 0; i <= GridCount; i++)
            {
                if (i != GridCount/2)
                {
                    gridVector.Add(new Vector3(-GridLength + i*GridWidth, 0, -GridLength));
                    gridVector.Add(new Vector3(-GridLength + i*GridWidth, 0, GridLength));
                }
            }
            using (DataStream vs = new DataStream(gridVector.ToArray(), true, true))
            {
                BufferDescription bufDesc = new BufferDescription
                {
                    BindFlags = BindFlags.VertexBuffer,
                    SizeInBytes = (int) vs.Length
                };
                vertexBuffer = new Buffer(RenderContext.DeviceManager.Device, vs, bufDesc);
            }
            vectorCount = gridVector.Count;
            List<float> axisVector = new List<float>();
            addAxisVector(axisVector, AxisLength, 0, 0, new Vector4(1, 0, 0, 1));
            addAxisVector(axisVector, 0, AxisLength, 0, new Vector4(0, 1, 0, 1));
            addAxisVector(axisVector, 0, 0, AxisLength, new Vector4(0, 0, 1, 1));
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
            ShaderSignature v = effect.GetTechniqueByIndex(0).GetPassByIndex(0).Description.Signature;
            layout = new InputLayout(RenderContext.DeviceManager.Device, v, MeasureGridLayout.VertexElements);
            v = effect.GetTechniqueByIndex(0).GetPassByIndex(1).Description.Signature;
            axisLayout = new InputLayout(RenderContext.DeviceManager.Device, v, AxisGridLayout.VertexElements);
            VertexCount = axisVectorCount + vectorCount;
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
        private static void addAxisVector(List<float> vertexList, float x, float y, float z, Vector4 color)
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