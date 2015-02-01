using System.Collections.Generic;
using System.IO;
using Assimp;
using Assimp.Configs;
using MMF.MME;
using SlimDX;
using SlimDX.Direct3D11;

namespace MMF.Model.Assimp
{
    /// <summary>
    /// Class AssimpModel.
    /// </summary>
    public class AssimpModel:IDrawable,ISubsetDivided
    {
        private Scene modelScene;

        private ISubresourceLoader loader;

        private RenderContext context;

        private List<ISubset> subsets=new List<ISubset>();

        private MMEEffectManager effectManager;

        private InputLayout layout;

        public AssimpModel(RenderContext context,string fileName)
        {
            this.FileName = Path.GetFileName(fileName);
            this.context = context;
            loader=new BasicSubresourceLoader(Path.GetDirectoryName(fileName));
            var importer = new AssimpContext();
            modelScene=importer.ImportFile(fileName,PostProcessSteps.Triangulate|PostProcessSteps.GenerateSmoothNormals);
            Visibility = true;
            Initialize();
        }

        /// <summary>
        /// AssimpModelを初期化します
        /// </summary>
        /// <param name="context"></param>
        /// <param name="fileName"></param>
        /// <param name="modelScene">PostProcessSteps.Triangulate|PostProcessSteps.GenerateNormalsを指定して読み込んだSceneである必要があります。</param>
        /// <param name="loader"></param>
        public AssimpModel(RenderContext context,string fileName, Scene modelScene,ISubresourceLoader loader)
        {
            this.modelScene = modelScene;
            this.context = context;
            this.FileName = fileName;
            this.loader = loader;
            Visibility = true;
            Initialize();
        }

        private void Initialize()
        {
            Transformer=new BasicTransformer();
            for (int i = 0; i < modelScene.Meshes.Count; i++)
            {
                subsets.Add(new AssimpSubset(context,loader,this, modelScene,i));
            }
            effectManager = MMEEffectManager.LoadFromResource(@"MMF.Resource.Shader.DefaultShader.fx", this, context, loader);
            layout = new InputLayout(context.DeviceManager.Device, effectManager.EffectFile.GetTechniqueByIndex(1).GetPassByIndex(0).Description.Signature, BasicInputLayout.VertexElements);
        }

        /// <summary>
        /// アンマネージ リソースの解放およびリセットに関連付けられているアプリケーション定義のタスクを実行します。
        /// </summary>
        /// <exception cref="System.NotImplementedException"></exception>
        public void Dispose()
        {
            if(layout!=null&&!layout.Disposed)layout.Dispose();
            effectManager.Dispose();
            foreach (var subset in subsets)
            {
                subset.Dispose();
            }
        }

        /// <summary>
        /// 表示されているかどうかを判定する変数
        /// </summary>
        /// <value><c>true</c> if visibility; otherwise, <c>false</c>.</value>
        public bool Visibility { get;set; }

        /// <summary>
        /// ファイル名を格納する変数
        /// </summary>
        /// <value>The name of the file.</value>
        public string FileName { get; private set; }

        /// <summary>
        /// サブセットの数
        /// </summary>
        /// <value>The subset count.</value>
        public int SubsetCount
        {
            get { return subsets.Count; }
        }

        /// <summary>
        /// 頂点数
        /// </summary>
        /// <value>The vertex count.</value>
        public int VertexCount { get; private set; }
        /// <summary>
        /// モデルを動かす際に使用するクラス
        /// </summary>
        /// <value>The transformer.</value>
        public ITransformer Transformer { get; private set; }
        /// <summary>
        /// モデルを描画するときに呼び出します。
        /// </summary>
        public void Draw()
        {
            effectManager.ApplyAllMatrixVariables();
            context.DeviceManager.Device.ImmediateContext.InputAssembler.InputLayout = layout;
            context.DeviceManager.Device.ImmediateContext.InputAssembler.PrimitiveTopology =
                PrimitiveTopology.TriangleList;
            effectManager.EffectFile.GetVariableBySemantic("BONETRANS")
                .AsMatrix()
                .SetMatrixArray(new Matrix[] {Matrix.Identity});
            foreach (var subset in subsets)
            {
                effectManager.ApplyAllMaterialVariables(subset.MaterialInfo);
                effectManager.ApplyEffectPass(subset, MMEEffectPassType.Object, (isubset) => isubset.Draw(context.DeviceManager.Device));
            }
        }

        /// <summary>
        /// モデルを更新するときに呼び出します
        /// </summary>
        public void Update()
        {
          
        }

        /// <summary>
        /// セルフシャドウ色
        /// </summary>
        /// <value>The color of the self shadow.</value>
        public Vector4 SelfShadowColor { get; set; }

        /// <summary>
        /// 地面影色
        /// </summary>
        /// <value>The color of the ground shadow.</value>
        public Vector4 GroundShadowColor { get; set; }
    }
}
