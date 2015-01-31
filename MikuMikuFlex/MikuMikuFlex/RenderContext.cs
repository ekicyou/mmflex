using System;
using System.Collections.Generic;
using System.Windows.Forms;
using MMF.DeviceManager;
using MMF.Light;
using MMF.Matricies;
using MMF.Matricies.Camera;
using MMF.Matricies.Projection;
using MMF.Matricies.World;
using MMF.Model;
using MMF.Motion;
using SlimDX;
using SlimDX.Direct2D;
using SlimDX.Direct3D11;
using FactoryType = SlimDX.DirectWrite.FactoryType;
using FillMode = SlimDX.Direct3D11.FillMode;

namespace MMF
{
    /// <summary>
    /// レンダリングに関する様々な情報を保持します。
    /// 3DCG描画をする際はこの参照が必要になることが多いです
    /// </summary>
    public class RenderContext : IDisposable
    {
        public event EventHandler Update = delegate { };

        /// <summary>
        /// コントロールを指定せずRenderContextを作成します
        /// </summary>
        /// <param name="deviceManager">使用するデバイスマネージャ．</param>
        /// <returns>作成されたRenderContext</returns>
        public static RenderContext CreateContext(IDeviceManager deviceManager = null)
        {
            RenderContext context = new RenderContext
            {
                DeviceManager = deviceManager
            };
            context.Initialize();
            return context;
        }

        /// <summary>
        /// コントロールと、ScreenContextの連想配列
        /// </summary>

        private Dictionary<Control,ScreenContext> screenContexts=new Dictionary<Control, ScreenContext>(); 


        /// <summary>
        /// 更新の必要なワールド
        /// 重い処理なので、必要のないときははずすべき
        /// </summary>

        public List<WorldSpace> UpdateRequireWorlds=new List<WorldSpace>(); 


        /// <summary>
        /// 現在描画スレッドで描画を実行しているコンテキスト
        /// </summary>
        public ITargetContext CurrentTargetContext { get; private set; }

        /// <summary>
        /// ライト管理クラス(作成中)
        /// </summary>
        public LightMatrixManager LightManager;

        /// <summary>

        /// 現在のインスタンスが使用しているデバイスマネージャを取得します．
        /// </summary>
        public IDeviceManager DeviceManager { get; private set; }
        /// <summary>
        /// デバイスマネージャの破棄をこのインスタンスで行うか否か．
        /// </summary>
        private bool disposeDeviceManager = false;


        /// <summary>
        ///     MME準拠の行列を管理するクラス
        /// </summary>
        public MatrixManager MatrixManager
        {
            get
            {
                return CurrentTargetContext.MatrixManager;
            }
        }

        /// <summary>
        ///     タイマー
        /// </summary>
        public MotionTimer Timer;


        internal List<IDisposable> Disposables=new List<IDisposable>(); 

        /// <summary>
        /// 現在のClearSetColorによる値
        /// </summary>
        public Color4 CurrentClearColor { get; set; }

        /// <summary>
        /// 現在のClearSetDepthによる値
        /// </summary>
        public float CurrentClearDepth { get;set; }

        /// <summary>
        /// 現在のRenderColorTarget0~7の値
        /// </summary>
        public RenderTargetView[] CurrentRenderColorTargets = new RenderTargetView[8];

        /// <summary>
        /// 現在のRenderDepthStencilTargetの値
        /// </summary>
        public DepthStencilView CurrentRenderDepthStencilTarget;

        /// <summary>
        /// ScreenContextがターゲットになっていないとnullになる
        /// </summary>
        public PanelObserver CurrentPanelObserver;

        /// <summary>
        /// 片面描画の際のラスタライザステート
        /// </summary>
        public RasterizerState CullingRasterizerState { get; private set; }

        /// <summary>
        /// 両面描画の際のラスタライザステート
        /// </summary>
        public RasterizerState NonCullingRasterizerState { get; private set; }

        /// <summary>
        /// Direct2Dのファクトリ
        /// </summary>
        public Factory D2DFactory { get; set; }

        /// <summary>
        /// DirectWriteのファクトリ
        /// </summary>
        public SlimDX.DirectWrite.Factory DWFactory { get; private set; }

        public BlendStateManager BlendingManager { get; private set; }

        /// <summary>
        /// コントロールと、ScreenContextの連想配列
        /// </summary>
        public Dictionary<Control, ScreenContext> ScreenContexts
        {
            get { return screenContexts; }
        }

        /// <summary>
        ///     コンストラクタ
        /// </summary>
        public RenderContext()
        {
        }

        public RenderContext(BasicGraphicDeviceManager deviceManager)
        {
            this.DeviceManager = deviceManager;
        }


        /// <summary>
        /// アンマネージ リソースの解放およびリセットに関連付けられているアプリケーション定義のタスクを実行します。
        /// </summary>
        public void Dispose()
        {
            foreach (var screenContext in screenContexts)
            {
                screenContext.Value.Dispose();
            }
            if (CullingRasterizerState != null && !CullingRasterizerState.Disposed) CullingRasterizerState.Dispose();
            if (NonCullingRasterizerState != null && !NonCullingRasterizerState.Disposed)
                NonCullingRasterizerState.Dispose();
            foreach (var disposable in Disposables)
            {
                disposable.Dispose();
            }
            if (D2DFactory != null && !D2DFactory.Disposed) D2DFactory.Dispose();
            if (DWFactory != null && !DWFactory.Disposed) DWFactory.Dispose();

            if (disposeDeviceManager)
                DeviceManager.Dispose();
        }

        public void Initialize()
        {
            InitializeDevices();
            Timer = new MotionTimer(this);
        }

        /// <summary>
        ///     RenderContextの初期化処理
        /// </summary>
        public ScreenContext Initialize(Control targetControl)
        {
            InitializeDevices();
            Timer = new MotionTimer(this);
            BlendingManager=new BlendStateManager(this);
            BlendingManager.SetBlendState(BlendStateManager.BlendStates.Alignment);
            //Matrixの初期化
            var matrixManager = InitializeMatricies();

            ScreenContext primaryContext = new ScreenContext(targetControl, this, matrixManager);
            
            screenContexts.Add(targetControl, primaryContext);
            CurrentTargetContext = primaryContext;
            //PhysicsManager=new PhysicsManager(this);
            ResetTargets();

            return primaryContext;
        }

        private MatrixManager InitializeMatricies()
        {
            BasicCamera Camera = new BasicCamera(new Vector3(0, 20, -40), new Vector3(0, 3, 0), new Vector3(0, 1, 0));
            BasicProjectionMatrixProvider Projection = new BasicProjectionMatrixProvider();
            Projection.InitializeProjection((float) Math.PI/4f, 1.618f, 1, 2000);
            MatrixManager matrixManager = new MatrixManager(new BasicWorldMatrixProvider(), Camera, Projection);
            LightManager = new LightMatrixManager(matrixManager);
            return matrixManager;
        }

        private void InitializeDevices()
        {
            InitializeMatricies();
            //3DCGの初期化
            if (DeviceManager == null)
            {
                disposeDeviceManager = true;
                DeviceManager = new BasicGraphicDeviceManager();
                DeviceManager.Load();
            }
            RasterizerStateDescription desc = new RasterizerStateDescription();
            desc.CullMode = CullMode.Back;
            desc.FillMode = FillMode.Solid;
           CullingRasterizerState = RasterizerState.FromDescription(DeviceManager.Device, desc);
            desc.CullMode = CullMode.None;
            NonCullingRasterizerState = RasterizerState.FromDescription(DeviceManager.Device, desc);

            //DirectWriteファクトリの作成
#if VSG_DEBUG
#else
            DWFactory = new SlimDX.DirectWrite.Factory(FactoryType.Isolated);
            D2DFactory = new Factory(SlimDX.Direct2D.FactoryType.Multithreaded, DebugLevel.Information);
#endif
        }

        /// <summary>
        /// 現在のスクリーンコンテキストに対してデフォルトのレンダリングターゲットを設定する
        /// </summary>
        private void ResetTargets()
        {
            CurrentRenderColorTargets[0] = CurrentTargetContext.RenderTargetView;
            CurrentRenderDepthStencilTarget = CurrentTargetContext.DepthTargetView;
            DeviceManager.Context.OutputMerger.SetTargets(CurrentRenderDepthStencilTarget,
                CurrentRenderColorTargets);
        }

        /// <summary>
        /// 現在のスクリーンコンテキストに対して画面をクリアする
        /// </summary>
        /// <param name="color"></param>
        public void ClearScreenTarget(Color4 color)
        {
            ResetTargets();
            DeviceManager.Context.ClearRenderTargetView(CurrentTargetContext.RenderTargetView,
                               color);
            DeviceManager.Context.ClearDepthStencilView(CurrentTargetContext.DepthTargetView,
                DepthStencilClearFlags.Depth, 1, 0);
        }

        /// <summary>
        /// ワールド空間を全てアップデートする
        /// </summary>
        public void UpdateWorlds()
        {
            foreach (var updateReqireWorld in UpdateRequireWorlds)
            {
                updateReqireWorld.UpdateAllDynamicTexture();
                foreach (var drawableGroup in updateReqireWorld.DrawableGroups)
                {
                    drawableGroup.ForEach(drawable=>drawable.Update());
                }
            }
            Update(this,new EventArgs());

        }

        /// <summary>
        /// 指定したコントロールを描画対象にします
        /// </summary>
        /// <param name="context"></param>
        public void SetRenderScreen(ITargetContext context)
        {
            CurrentTargetContext = context;
            context.SetViewport();
        }

        public ScreenContext CreateScreenContext(Control control)
        {
            BasicCamera camera = new BasicCamera(new Vector3(0, 20, -40), new Vector3(0, 3, 0), new Vector3(0, 1, 0));
            BasicProjectionMatrixProvider projection = new BasicProjectionMatrixProvider();
            projection.InitializeProjection((float)Math.PI / 4f, 1.618f, 1, 200);
            MatrixManager matrixManager = new MatrixManager(new BasicWorldMatrixProvider(), camera, projection);
            ScreenContext context=new ScreenContext(control,this,matrixManager);
            screenContexts.Add(control,context);

            return context;
        }

        /// <summary>
        /// テクスチャを生成します
        /// </summary>
        /// <returns></returns>
        public Texture2D CreateTexture2D(Texture2DDescription desc)
        {
            return new Texture2D(DeviceManager.Device,desc);
        }
    }
}