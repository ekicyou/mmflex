//#define VSG_DEBUG//VS2012のグラフィックデバッガ使う際はDirect2D使うとエラーになるので、コメントアウトをはずす。BasicGraphicDeviceManagerも同様にする。

using System;
using System.ComponentModel;
using System.Drawing;
using System.Windows.Forms;
using MMF.DeviceManager;
using MMF.Sprite;
using SlimDX.DXGI;

namespace MMF.Controls.Forms
{
    /// <summary>
    ///     Direct2D/DirectWriteを利用する場合のRenderForm
    /// </summary>
    public abstract partial class D2DSupportedRenderForm : RenderForm
    {
        private bool IsInitialized;

#if VSG_DEBUG
#else

        /// <summary>
        ///     D2D描画に利用するスプライト
        /// </summary>
        public D2DSpriteBatch SpriteBatch { get; private set; }
#endif

        public D2DSupportedRenderForm()
        {
            InitializeComponent();
        }

        public D2DSupportedRenderForm(RenderContext context) : base(context)
        {
        }

        public D2DSupportedRenderForm(BasicGraphicDeviceManager deviceManager) : base(deviceManager)
        {
        }

        protected override void OnLoad(EventArgs e)
        {
            #region デザインモード時処理

            if (DesignMode)
            {
                var label = new Label();
                label.Text = "D2DSuppurtedRenderForm\n*デザインモードでは描画できません。\n*ウィンドウの大きさ、タイトルなどはデザインビューからも変更可能です。";
                label.Dock = DockStyle.Fill;
                label.TextAlign = ContentAlignment.MiddleCenter;
                label.Font = new Font("Meiriyo", 30);
                Controls.Add(label);
                return;
            }

            #endregion

            base.OnLoad(e);
#if VSG_DEBUG
#else
            RenderContext.SetRenderScreen(ScreenContext);
            SpriteBatch = new D2DSpriteBatch(RenderContext);
#endif
            ClientSizeChanged += D2DSupportedRenderForm_ClientSizeChanged;
            IsInitialized = true;
        }

        private void D2DSupportedRenderForm_ClientSizeChanged(object sender, EventArgs e)
        {
            if (DesignMode) return;
#if VSG_DEBUG
#else
            if (SpriteBatch != null)
            {
                RenderContext.SetRenderScreen(ScreenContext);
                SpriteBatch.Resize();
            }
#endif
        }

        protected override void OnClosing(CancelEventArgs e)
        {
            base.OnClosing(e);
        }


        protected override void OnClosed(EventArgs e)
        {
            base.OnClosed(e);
        }


        public override void Render()
        {
            if (!IsInitialized || !Visible) return;
            RenderContext.SetRenderScreen(ScreenContext);
            ScreenContext.MoveCameraByCameraMotionProvider();
            ClearViews();
            RenderContext.Timer.TickUpdater();
            FpsCounter.CountFrame();
            ScreenContext.WorldSpace.DrawAllResources(ScreenContext.HitChekcer);
#if VSG_DEBUG
#else
            SpriteBatch.Begin();
            RenderSprite();
            SpriteBatch.End();
#endif
            ScreenContext.SwapChain.Present(0, PresentFlags.None);
            OnPresented();
        }

        protected override void OnHandleDestroyed(EventArgs e)
        {
            base.OnHandleDestroyed(e);
#if VSG_DEBUG

#else
            if (SpriteBatch != null) SpriteBatch.Dispose();
#endif
        }

        protected abstract void RenderSprite();
    }
}