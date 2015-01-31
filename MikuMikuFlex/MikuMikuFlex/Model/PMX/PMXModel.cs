// ***********************************************************************
// Assembly         : MikuMikuFlex
// Author           : Lime
// Created          : 01-17-2014
//
// Last Modified By : Lime
// Last Modified On : 02-02-2014
// ***********************************************************************
// <copyright file="MMDModel.cs" company="MMF Development Team">
//     Copyright (c) MMF Development Team. All rights reserved.
// </copyright>
// <summary></summary>
// ***********************************************************************

using System.Diagnostics;
using System.IO;
using MMDFileParser.PMXModelParser;
using MMF.Bone;
using MMF.Controls.Forms;
using MMF.MME;
using MMF.Morph;
using MMF.Motion;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;
using Device = SlimDX.Direct3D11.Device;

namespace MMF.Model.PMX
{
    /// <summary>
    /// Class MMDModel.
    /// </summary>
    public class PMXModel :ISubsetDivided,IMovable,IEdgeDrawable,IGroundShadowDrawable
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="PMXModel"/> class.
        /// </summary>
        /// <param name="modeldata">The modeldata.</param>
        /// <param name="subResourceLoader">The sub resource loader.</param>
        /// <param name="filename">The filename.</param>
        public PMXModel(ModelData modeldata, ISubresourceLoader subResourceLoader, string filename)
        {
            Model = modeldata;
            SubresourceLoader = subResourceLoader;
            Transformer = new BasicTransformer();
            SubsetManager = new PMXSubsetManager(this,modeldata);
            //PhongConstantBuffer = new PhongShadingConstantBufferManager();
            ToonManager = new PMXToonTextureManager();
            SelfShadowColor = new Vector4(0, 0, 0, 1);
            GroundShadowColor = new Vector4(0, 0, 0, 1);
            FileName = filename;
            Visibility = true;
        }

        /// <summary>
        /// Prevents a default instance of the <see cref="PMXModel"/> class from being created.
        /// </summary>
        private PMXModel()
        {
            
        }

        /// <summary>
        /// テクスチャなどのリソースのパスを解決するインターフェース
        /// </summary>
        /// <value>The subresource loader.</value>
        public ISubresourceLoader SubresourceLoader { get; private set; }

        /// <summary>
        /// 読み込み対象のモデル
        /// </summary>
        /// <value>The model.</value>
        public ModelData Model { get; private set; }


        /// <summary>
        /// バッファ管理インターフェース
        /// </summary>
        /// <value>The buffer manager.</value>
        public IBufferManager BufferManager { get; private set; }

        /// <summary>
        /// サブセット管理インターフェース
        /// </summary>
        /// <value>The subset manager.</value>
        public ISubsetManager SubsetManager { get; private set; }

        /// <summary>
        /// Gets the morphmanager.
        /// </summary>
        /// <value>The morphmanager.</value>
        public IMorphManager Morphmanager { get; private set; }


        /// <summary>
        /// 描画に利用するMMEエフェクト
        /// </summary>
        /// <value>The effect.</value>
        public MMEEffectManager Effect { get; private set; }



       // public PhongShadingConstantBufferManager PhongConstantBuffer { get; private set; }

        /// <summary>
        /// トゥーンを管理してくれるインターフェース
        /// </summary>
        /// <value>The toon manager.</value>
        public IToonTextureManager ToonManager { get; private set; }

        /// <summary>
        /// レンダーコンテキスト
        /// </summary>
        /// <value>The render context.</value>
        public RenderContext RenderContext { get; private set; }

        /// <summary>
        /// Gets or sets the z plot pass.
        /// </summary>
        /// <value>The z plot pass.</value>
        private EffectPass ZPlotPass { get; set; }

        /// <summary>
        /// 初期化判定
        /// </summary>
        /// <value><c>true</c> if this instance is initialized; otherwise, <c>false</c>.</value>
        private bool IsInitialized { get; set; }

        #region IDrawableインターフェース実装部

        /// <summary>
        /// The ground shadow color
        /// </summary>
        private Vector4 groundShadowColor;
        //private PhongShadingConstantBufferInputLayout phongBuffer;
        /// <summary>
        /// The self shadow color
        /// </summary>
        private Vector4 selfShadowColor;
        /// <summary>
        /// 表示されているかどうかを判定する変数
        /// </summary>
        /// <value><c>true</c> if visibility; otherwise, <c>false</c>.</value>
        public bool Visibility { get; set; }
        /// <summary>
        /// ファイル名を格納する変数
        /// </summary>
        /// <value>The name of the file.</value>
        public string FileName { get; set; }

        /// <summary>
        /// Loads from effect file.
        /// </summary>
        /// <param name="filePath">The file path.</param>
        public void LoadFromEffectFile(string filePath)
        {
            Effect = MMEEffectManager.Load(filePath, this, RenderContext, SubresourceLoader);
        }

        /// <summary>
        /// サブセットの数
        /// </summary>
        /// <value>The subset count.</value>
        public int SubsetCount
        {
            get
            {
                return SubsetManager.SubsetCount;
            }
        }

        /// <summary>
        /// 頂点数
        /// </summary>
        /// <value>The vertex count.</value>
        public int VertexCount
        {
            get
            {
                return BufferManager.VerticiesCount;
            }
        }

        /// <summary>
        /// モデルを描画するときに呼び出します。
        /// </summary>
        public virtual void Draw()
        {
            Effect.ApplyAllMatrixVariables();
            Skinning.ApplyEffect(Effect.EffectFile);
            RenderContext.DeviceManager.Device.ImmediateContext.InputAssembler.SetVertexBuffers(0,
                new VertexBufferBinding(BufferManager.VertexBuffer, BasicInputLayout.SizeInBytes, 0));
            RenderContext.DeviceManager.Device.ImmediateContext.InputAssembler.SetIndexBuffer(
                BufferManager.IndexBuffer, Format.R32_UInt, 0);
            RenderContext.DeviceManager.Device.ImmediateContext.InputAssembler.InputLayout = BufferManager.VertexLayout;
            RenderContext.DeviceManager.Device.ImmediateContext.InputAssembler.PrimitiveTopology =
                PrimitiveTopology.TriangleList;

            SubsetManager.DrawAll();
        }


        /// <summary>
        /// Draws the edge.
        /// </summary>
        public void DrawEdge()
        {
            Effect.ApplyAllMatrixVariables();
            Skinning.ApplyEffect(Effect.EffectFile);
            RenderContext.DeviceManager.Device.ImmediateContext.InputAssembler.SetVertexBuffers(0,
                new VertexBufferBinding(BufferManager.VertexBuffer, BasicInputLayout.SizeInBytes, 0));
            RenderContext.DeviceManager.Device.ImmediateContext.InputAssembler.SetIndexBuffer(
                BufferManager.IndexBuffer, Format.R32_UInt, 0);
            RenderContext.DeviceManager.Device.ImmediateContext.InputAssembler.InputLayout = BufferManager.VertexLayout;
            RenderContext.DeviceManager.Device.ImmediateContext.InputAssembler.PrimitiveTopology =
                PrimitiveTopology.TriangleList;
            SubsetManager.DrawEdges();
        }

        /// <summary>
        /// Draws the ground shadow.
        /// </summary>
        public void DrawGroundShadow()
        {
            Effect.ApplyAllMatrixVariables();
            Skinning.ApplyEffect(Effect.EffectFile);
            RenderContext.DeviceManager.Device.ImmediateContext.InputAssembler.SetVertexBuffers(0,
                new VertexBufferBinding(BufferManager.VertexBuffer, BasicInputLayout.SizeInBytes, 0));
            RenderContext.DeviceManager.Device.ImmediateContext.InputAssembler.SetIndexBuffer(
                BufferManager.IndexBuffer, Format.R32_UInt, 0);
            RenderContext.DeviceManager.Device.ImmediateContext.InputAssembler.InputLayout = BufferManager.VertexLayout;
            RenderContext.DeviceManager.Device.ImmediateContext.InputAssembler.PrimitiveTopology =
                PrimitiveTopology.TriangleList;
            SubsetManager.DrawGroundShadow();
        }

        /// <summary>
        /// モデルを更新するときに呼び出します
        /// </summary>
        public void Update()
        {
            if(BufferManager!=null)BufferManager.RecreateVerticies();
            Morphmanager.UpdateFrame();
            Skinning.UpdateSkinning(Morphmanager);
            foreach (var pmxSubset in SubsetManager.Subsets)
            {
                pmxSubset.MaterialInfo.UpdateMaterials();
            }
        }

        /// <summary>
        /// 1000/30msごとに呼び出され、フレームを更新します。
        /// </summary>
        public virtual void ApplyMove()
        {
        }
        /// <summary>
        /// Loads the specified render context.
        /// </summary>
        /// <param name="renderContext">The render context.</param>
        public void Load(RenderContext renderContext)
        {
            Stopwatch sw = new Stopwatch();
            sw.Start();
            RenderContext = renderContext;
            Device device = RenderContext.DeviceManager.Device;
            if (!IsInitialized)
            {
//モデルの読み込みなどはリセットしない
                //シェーダーの読み込み
                ToonManager.Initialize(RenderContext, SubresourceLoader);
                SubsetManager.Initialze(RenderContext, Effect, SubresourceLoader,ToonManager);
                Effect = InitializeEffect();
                SubsetManager.ResetEffect(Effect);

                //定数バッファ
                //PhongConstantBuffer.Initialize(device, Effect.EffectFile,
                //    PhongShadingConstantBufferInputLayout.SizeInBytes, new PhongShadingConstantBufferInputLayout());
                ZPlotPass = Effect.EffectFile.GetTechniqueByIndex(1).GetPassByIndex(0);
                InitializeBuffers(device);
                Skinning = InitializeSkinning();
                Morphmanager=new PMXMorphManager(this);
                IsInitialized = true;
            }
            MotionManager = InitializeMotionManager();
            InitializeOther(device);
            sw.Stop();
            Trace.WriteLine(sw.ElapsedMilliseconds + "ms");
        }


        //public Task LoadAsync(RenderContext renderContext)
        //{
        //    Task task=new Task(() => Load(renderContext));
        //    task.Start();
        //    return task;
        //}


        #endregion

        #region 初期化系

        /// <summary>
        /// Loads the effect.
        /// </summary>
        /// <param name="effectFile">The effect file.</param>
        public void LoadEffect(string effectFile)
        {
            if (Effect != null) Effect.Dispose();
            if (string.IsNullOrEmpty(effectFile))
            {
                Effect = MMEEffectManager.Load(@"Shader\DefaultShader.fx", this, RenderContext, SubresourceLoader);
            }
            else
            {
                Effect = MMEEffectManager.Load(effectFile, this, RenderContext, SubresourceLoader);
            }
            SubsetManager.ResetEffect(Effect);
        }

        /// <summary>
        /// Initializes the skinning.
        /// </summary>
        /// <returns>ISkinningProvider.</returns>
        protected virtual ISkinningProvider InitializeSkinning()
        {
            PMXSkeleton skel=new PMXSkeleton(Model);
            return skel;
        }

        /// <summary>
        /// Initializes the motion manager.
        /// </summary>
        /// <returns>IMotionManager.</returns>
        protected virtual IMotionManager InitializeMotionManager()
        {
            BasicMotionManager mm = new BasicMotionManager(RenderContext);
            mm.Initialize(Model,Morphmanager,Skinning,BufferManager);
            ((PMXSkeleton)Skinning).KinematicsProviders.Insert(0,mm);
            return mm;
        }

        /// <summary>
        /// Initializes the effect.
        /// </summary>
        /// <returns>MMEEffectManager.</returns>
        protected virtual MMEEffectManager InitializeEffect()
        {
            return MMEEffectManager.LoadFromResource(@"MMF.Resource.Shader.DefaultShader.fx", this, RenderContext, SubresourceLoader);
        }

        /// <summary>
        /// Initializes the other.
        /// </summary>
        /// <param name="device">The device.</param>
        protected virtual void InitializeOther(Device device)
        {
        }

        /// <summary>
        /// Initializes the buffers.
        /// </summary>
        /// <param name="device">The device.</param>
        protected virtual void InitializeBuffers(Device device)
        {
            BufferManager = new PMXModelBufferManager();
            BufferManager.Initialize(Model, device, Effect.EffectFile);
        }

        #endregion

        /// <summary>
        /// セルフシャドウの色
        /// </summary>
        /// <value>The color of the self shadow.</value>
        public Vector4 SelfShadowColor
        {
            get { return selfShadowColor; }
            set
            {
                selfShadowColor = value;
            }
        }

        /// <summary>
        /// 地面影の色
        /// </summary>
        /// <value>The color of the ground shadow.</value>
        public Vector4 GroundShadowColor
        {
            get { return groundShadowColor; }
            set
            {
                groundShadowColor = value;
            }
        }

        /// <summary>
        /// モデルの位置や回転量を管理するインターフェース
        /// </summary>
        /// <value>The transformer.</value>
        public ITransformer Transformer { get; private set; }

        /// <summary>
        /// アンマネージ リソースの解放およびリセットに関連付けられているアプリケーション定義のタスクを実行します。
        /// </summary>
        public virtual void Dispose()
        {
            Effect.Dispose();
            BufferManager.Dispose();
            ToonManager.Dispose();
            SubsetManager.Dispose();
        }

        /// <summary>
        /// モーション管理クラス
        /// </summary>
        /// <value>The motion manager.</value>
        public IMotionManager MotionManager { get; private set; }

        /// <summary>
        /// スキニングのインターフェース
        /// </summary>
        /// <value>The skinning.</value>
        public ISkinningProvider Skinning { get; private set; }

        #region 静的初期化便利メソッド

        
        #region FromFile
        /// <summary>
        /// モデルファイルを開く
        /// </summary>
        /// <param name="filePath">PMXのファイルパス、テクスチャは同じフォルダから読み込まれる</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static PMXModel FromFile(string filePath)
        {
            string folder = Path.GetDirectoryName(filePath);
            return FromFile(filePath, folder);
        }

        /// <summary>
        /// モデルファイルを開く
        /// </summary>
        /// <param name="filePath">PMXのファイルパス</param>
        /// <param name="textureFolder">テクスチャを読み込むフォルダ</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static PMXModel FromFile(string filePath, string textureFolder)
        {
            return FromFile(filePath, new BasicSubresourceLoader(textureFolder));
        }

        /// <summary>
        /// モデルファイルを開く
        /// </summary>
        /// <param name="filePath">PMXのファイルパス</param>
        /// <param name="loader">テクスチャのパス解決インターフェース</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static PMXModel FromFile(string filePath, ISubresourceLoader loader)
        {
            using (FileStream fs = File.OpenRead(filePath))
            {
                return new PMXModel(ModelData.GetModel(fs),loader, Path.GetFileName(filePath));
            }
        }
        #endregion

        #region OpenLoad

        /// <summary>
        /// 開いて初期化
        /// </summary>
        /// <param name="filePath">PMXのファイルパス</param>
        /// <param name="context">レンダリングコンテキスト</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static PMXModel OpenLoad(string filePath,RenderContext context)
        {
            PMXModel model = FromFile(filePath);
            model.Load(context);
            return model;
        }

        /// <summary>
        /// 開いて初期化
        /// </summary>
        /// <param name="filePath">PMXファイルのパス</param>
        /// <param name="textureFolder">テクスチャのフォルダ</param>
        /// <param name="context">レンダリングコンテキスト</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static PMXModel OpenLoad(string filePath, string textureFolder, RenderContext context)
        {
            PMXModel model = FromFile(filePath, textureFolder);
            model.Load(context);
            return model;
        }

        /// <summary>
        /// 開いて初期化
        /// </summary>
        /// <param name="filePath">PMXファイルのパス</param>
        /// <param name="loader">テクスチャなどのパスの解決インターフェース</param>
        /// <param name="context">レンダリングコンテキスト</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static PMXModel OpenLoad(string filePath, ISubresourceLoader loader, RenderContext context)
        {
            PMXModel model = FromFile(filePath, loader);
            model.Load(context);
            return model;
        }

        /// <summary>
        /// 開いて初期化
        /// </summary>
        /// <param name="filePath">PMXのファイルパス</param>
        /// <param name="panel">レンダーパネル</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static PMXModel OpenLoad(string filePath, RenderControl panel)
        {
            PMXModel model = FromFile(filePath);
            model.Load(panel.RenderContext);
            return model;
        }

        /// <summary>
        /// 開いて初期化
        /// </summary>
        /// <param name="filePath">PMXファイルのパス</param>
        /// <param name="textureFolder">テクスチャのフォルダ</param>
        /// <param name="panel">レンダーパネル</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static PMXModel OpenLoad(string filePath, string textureFolder, RenderControl panel)
        {
            PMXModel model = FromFile(filePath, textureFolder);
            model.Load(panel.RenderContext);
            return model;
        }

        /// <summary>
        /// 開いて初期化
        /// </summary>
        /// <param name="filePath">PMXファイルのパス</param>
        /// <param name="loader">テクスチャなどのパスの解決インターフェース</param>
        /// <param name="panel">レンダリングコンテキスト</param>
        /// <returns>MMDModel.</returns>
        public static PMXModel OpenLoad(string filePath, ISubresourceLoader loader, RenderControl panel)
        {
            PMXModel model = FromFile(filePath, loader);
            model.Load(panel.RenderContext);
            return model;
        }
        #endregion

        /*
        #region OpenLoadAsync
        /// <summary>
        /// 開いて初期化(非同期)
        /// </summary>
        /// <param name="filePath">PMXのファイルパス</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static Task<MMDModel> OpenLoadAsync(string filePath, RenderContext context)
        {
            Task<MMDModel> task=new Task<MMDModel>(() =>
            {
                MMDModel model = FromFile(filePath);
                model.Load(context);
                return model;
            });
            task.Start();
            return task;
        }

        /// <summary>
        /// 開いて初期化(非同期)
        /// </summary>
        /// <param name="filePath">PMXファイルのパス</param>
        /// <param name="textureFolder">テクスチャのフォルダ</param>
        /// <param name="context">レンダリングコンテキスト</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static Task<MMDModel> OpenLoadAsync(string filePath, string textureFolder, RenderContext context)
        {
            Task<MMDModel> task = new Task<MMDModel>(() =>
            {
                MMDModel model = FromFile(filePath, textureFolder);
                model.Load(context);
                return model;
            });
            task.Start();
            return task;
        }

        /// <summary>
        /// 開いて初期化(非同期)
        /// </summary>
        /// <param name="filePath">PMXファイルのパス</param>
        /// <param name="loader">テクスチャなどのパスの解決インターフェース</param>
        /// <param name="context">レンダリングコンテキスト</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static Task<MMDModel> OpenLoadAsync(string filePath, ISubresourceLoader loader, RenderContext context)
        {
            Task<MMDModel> task = new Task<MMDModel>(() =>
            {
                MMDModel model = FromFile(filePath, loader);
                model.Load(context);
                return model;
            });
            task.Start();
            return task;

        }

        /// <summary>
        /// 開いて初期化(非同期)
        /// </summary>
        /// <param name="filePath">PMXのファイルパス</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static Task<MMDModel> OpenLoadAsync(string filePath, RenderControl panel)
        {
            Task<MMDModel> task = new Task<MMDModel>(() =>
            {
                MMDModel model = FromFile(filePath);
                model.Load(panel.RenderContext);
                return model;
            });
            task.Start();
            return task;
        }

        /// <summary>
        /// 開いて初期化(非同期)
        /// </summary>
        /// <param name="filePath">PMXファイルのパス</param>
        /// <param name="textureFolder">テクスチャのフォルダ</param>
        /// <param name="panel">レンダーパネル</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static Task<MMDModel> OpenLoadAsync(string filePath, string textureFolder, RenderControl panel)
        {
            Task<MMDModel> task = new Task<MMDModel>(() =>
            {
                MMDModel model = FromFile(filePath, textureFolder);
                model.Load(panel.RenderContext);
                return model;
            });
            task.Start();
            return task;

        }

        /// <summary>
        /// 開いて初期化(非同期)
        /// </summary>
        /// <param name="filePath">PMXファイルのパス</param>
        /// <param name="loader">テクスチャなどのパスの解決インターフェース</param>
        /// <param name="panel">レンダーパネル</param>
        /// <returns>MMDModelのインスタンス</returns>
        public static Task<MMDModel> OpenLoadAsync(string filePath, ISubresourceLoader loader, RenderControl panel)
        {
            Task<MMDModel> task = new Task<MMDModel>(() =>
            {
                MMDModel model = FromFile(filePath, loader);
                model.Load(panel.RenderContext);
                return model;
            });
            task.Start();
            return task;
        }
        #endregion
        */

        #endregion


    }
}