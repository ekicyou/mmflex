using System;
using System.Drawing;
using System.Windows.Forms;
using MMF.DeviceManager;
using MMF.Utility;
using SlimDX;
using SlimDX.DXGI;

namespace MMF.Controls.Forms
{
    /// <summary>
    ///     レンダリング対象のフォームのベースになるクラス
    /// </summary>
    public partial class RenderForm : Form
    {
        private bool IsInitialized;

        /// <summary>
        ///     Initializes a new instance of the <see cref="RenderForm" /> class.
        /// </summary>
        public RenderForm()
        {
            InitializeComponent();
            BackgroundColor = new Vector3(0.2f, 0.4f, 0.8f);
        }

        /// <summary>
        ///     レンダーコンテキストが存在する場合
        /// </summary>
        /// <param name="context"></param>
        public RenderForm(RenderContext context) : this()
        {
            RenderContext = context;
        }

        /// <summary>
        ///     デバイスの作成をカスタマイズしたい場合
        /// </summary>
        /// <param name="deviceManager"></param>
        public RenderForm(BasicGraphicDeviceManager deviceManager) : this()
        {
            RenderContext = new RenderContext(deviceManager);
        }

        /// <summary>
        ///     フォームの初期化に利用したレンダーコンテキスト
        /// </summary>
        public RenderContext RenderContext { get; private set; }

        /// <summary>
        ///     スクリーンのコンテキスト
        /// </summary>
        public ScreenContext ScreenContext { get; private set; }

        /// <summary>
        ///     描画対象のワールド空間
        /// </summary>
        public WorldSpace WorldSpace
        {
            get
            {
                if (DesignMode) return null;
                return ScreenContext.WorldSpace;
            }
            set
            {
                if (DesignMode) return;
                if (value == null)
                {
                    throw new InvalidOperationException("ワールド空間にnullは指定できません");
                }
                ScreenContext.WorldSpace = value;
            }
        }


        /// <summary>
        ///     FPS計測クラス
        /// </summary>
        public FPSCounter FpsCounter { get; private set; }

        /// <summary>
        ///     レンダーターゲットのクリア色
        /// </summary>
        public Vector3 BackgroundColor { get; set; }

        /// <summary>
        ///     ペイントループによる描画を実行するか
        ///     MessagePump.RunによってRenderメソッドを呼ぶことができない場合、trueにするとフォームの中でループを実行する
        /// </summary>
        public bool DoOnPaintLoop { get; set; }

        /// <summary>
        ///     <see cref="E:System.Windows.Forms.Form.Load" /> イベントを発生させます。
        /// </summary>
        /// <param name="e">イベント データを格納している <see cref="T:System.EventArgs" />。</param>
        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);

            #region デザインモード時処理

            if (DesignMode)
            {
                var label = new Label();
                label.Text = "RenderForm\n*デザインモードでは描画できません。\n*ウィンドウの大きさ、タイトルなどはデザインビューからも変更可能です。";
                label.Dock = DockStyle.Fill;
                label.TextAlign = ContentAlignment.MiddleCenter;
                label.Font = new Font("Meiriyo", 30);
                Controls.Add(label);
                return;
            }

            #endregion

            if (RenderContext == null)
            {
                RenderContext = new RenderContext();
                ScreenContext = RenderContext.Initialize(this);
                RenderContext.UpdateRequireWorlds.Add(WorldSpace);
            }
            else //RenderContextがすでにあるばあい
            {
                ScreenContext = RenderContext.CreateScreenContext(this);
            }
            FpsCounter = new FPSCounter();
            FpsCounter.Start();
            ClientSizeChanged += RenderForm_ClientSizeChanged;
            IsInitialized = true;
        }

        /// <summary>
        ///     Handles the ClientSizeChanged event of the RenderForm control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="EventArgs" /> instance containing the event data.</param>
        private void RenderForm_ClientSizeChanged(object sender, EventArgs e)
        {
            if (ScreenContext != null && RenderContext.DeviceManager != null)
            {
                ScreenContext.Resize();
                ScreenContext.MatrixManager.ProjectionMatrixManager.AspectRatio = (float) Width/Height;
            }
        }

        public virtual void OnUpdated()
        {
        }


        /// <summary>
        ///     コントロールの背景を描画します。
        /// </summary>
        /// <param name="e">イベント データを格納している <see cref="T:System.Windows.Forms.PaintEventArgs" />。</param>
        protected override void OnPaintBackground(PaintEventArgs e)
        {
            if (DesignMode) base.OnPaintBackground(e);
        }

        /// <summary>
        ///     DoOnPaintLoop=trueのとき用
        /// </summary>
        /// <param name="e">A <see cref="T:System.Windows.Forms.PaintEventArgs" /> that contains the event data.</param>
        protected override void OnPaint(PaintEventArgs e)
        {
            if (RenderContext != null && DoOnPaintLoop && !DesignMode)
            {
                Render();
                Invalidate();
            }
        }

        /// <summary>
        ///     画面をクリアします
        /// </summary>
        protected virtual void ClearViews()
        {
            RenderContext.ClearScreenTarget(new Color4(BackgroundColor));
        }


        /// <summary>
        ///     レンダリング
        /// </summary>
        public virtual void Render()
        {
            if (!IsInitialized || !Visible) return;
            RenderContext.SetRenderScreen(ScreenContext);
            ScreenContext.SetPanelObserver();
            ScreenContext.MoveCameraByCameraMotionProvider();
            RenderContext.Timer.TickUpdater();
            FpsCounter.CountFrame();
            ClearViews();
            ScreenContext.WorldSpace.DrawAllResources(ScreenContext.HitChekcer);
            ScreenContext.SwapChain.Present(0, PresentFlags.None);
            OnPresented();
        }


        protected virtual void OnPresented()
        {
        }

        protected override void OnClosed(EventArgs e)
        {
            base.OnClosed(e);
        }

        protected override void OnHandleDestroyed(EventArgs e)
        {
            base.OnHandleDestroyed(e);
            RenderContext.ScreenContexts.Remove(this);
            ScreenContext.Dispose();
            if (RenderContext.ScreenContexts.Count == 0) RenderContext.Dispose();
        }
    }
}