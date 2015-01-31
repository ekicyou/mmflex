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
    ///     レンダリングビューの基本クラス
    /// </summary>
    public class RenderControl : UserControl
    {
        /// <summary>
        ///     レンダリングコンテキスト
        ///     モデル表示など、様々な場所で必要となる
        /// </summary>
        public RenderContext RenderContext { get; private set; }

        public ScreenContext ScreenContext { get; private set; }

        public WorldSpace WorldSpace
        {
            get { return ScreenContext.WorldSpace; }
            set
            {
                if (value == null)
                {
                    throw new InvalidOperationException("ワールド空間にnullは指定できません。");
                }
                ScreenContext.WorldSpace = value;
            }
        }

        /// <summary>
        ///     バックグラウンドのカラー
        /// </summary>
        public Color3 BackgroundColor { get; set; }

        /// <summary>
        ///     FPSカウンタ
        /// </summary>
        public FPSCounter FpsCounter { get; private set; }

        /// <summary>
        ///     初期化したかどうかの判定
        /// </summary>
        public bool IsInitialized { get; private set; }

        /// <summary>
        ///     初期化処理
        /// </summary>
        /// <param name="context"></param>
        public virtual void Initialize(RenderContext context = null)
        {
            if (context == null)
            {
                RenderContext = new RenderContext();
                ScreenContext = RenderContext.Initialize(this);
                RenderContext.UpdateRequireWorlds.Add(WorldSpace);
            }
            else
            {
                RenderContext = context;
                ScreenContext = context.CreateScreenContext(this);
            }
            FpsCounter = new FPSCounter();
            FpsCounter.Start();
            IsInitialized = true;
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            if (DesignMode)
            {
                var label = new Label();
                label.Dock = DockStyle.Fill;
                label.TextAlign = ContentAlignment.MiddleCenter;
                label.Text =
                    string.Format(
                        "{0}\n*デザインモード時は描画できません。\n*必ずFormのOnLoadでInitializeメソッドを呼び出してください。\n*別のコントロールで利用したRenderContextを利用する場合はInitializeの第一引数に与えてください",
                        GetType());
                Controls.Add(label);
            }
        }

        protected override void OnClientSizeChanged(EventArgs e)
        {
            base.OnClientSizeChanged(e);
            if (!DesignMode && ScreenContext != null)
            {
                ScreenContext.Resize();
                ScreenContext.MatrixManager.ProjectionMatrixManager.AspectRatio = (float) Width/Height;
            }
        }


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
            ScreenContext.MoveCameraByCameraMotionProvider();
            RenderContext.Timer.TickUpdater();
            FpsCounter.CountFrame();
            ClearViews();
            ScreenContext.WorldSpace.DrawAllResources(ScreenContext.HitChekcer);
            ScreenContext.SwapChain.Present(0, PresentFlags.None);
        }

        /// <summary>
        ///     <see cref="T:System.Windows.Forms.Control" /> とその子コントロールが使用しているアンマネージ リソースを解放します。オプションで、マネージ リソースも解放します。
        /// </summary>
        /// <param name="disposing">マネージ リソースとアンマネージ リソースの両方を解放する場合は true。アンマネージ リソースだけを解放する場合は false。</param>
        protected override void Dispose(bool disposing)
        {
            base.Dispose(disposing);
            if (RenderContext != null) RenderContext.Dispose();
            RenderContext.ScreenContexts.Remove(this);
            ScreenContext.Dispose();
            if (RenderContext.ScreenContexts.Count == 0) RenderContext.Dispose();
        }
    }
}