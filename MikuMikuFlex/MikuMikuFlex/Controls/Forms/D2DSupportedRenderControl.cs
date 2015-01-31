#define VSG_DEBUG
using System;
using MMF.Sprite;
using SlimDX.DXGI;

namespace MMF.Controls.Forms
{
    /// <summary>
    ///     Direct2D/DirectWriteによる2Dを3DCG画面に重ねる描画をするクラス
    /// </summary>
    public class D2DSupportedRenderControl : RenderControl
    {
        /// <summary>
        ///     スプライト表示に利用するクラスを取得します
        /// </summary>
        /// <value>
        ///     取得する値は、<see cref="D2DSpriteBatch" />をご覧ください。
        /// </value>
        public D2DSpriteBatch SpriteBatch { get; private set; }

        /// <summary>
        ///     スプライトの更新処理を行うハンドラです
        /// </summary>
        /// <value>
        ///     void XXX(D2DSpriteBatch hoge);型のデリゲート
        /// </value>
        public Action<D2DSpriteBatch> DrawSpriteHandler { get; set; }

        public override void Render()
        {
            if (!IsInitialized || !Visible) return;
            RenderContext.SetRenderScreen(ScreenContext);
            ScreenContext.MoveCameraByCameraMotionProvider();
            RenderContext.Timer.TickUpdater();
            FpsCounter.CountFrame();
            ClearViews();
            ScreenContext.WorldSpace.DrawAllResources(ScreenContext.HitChekcer);

#if VSG_DEBUG
#else
            SpriteBatch.Begin();
            if (DrawSpriteHandler != null) DrawSpriteHandler(SpriteBatch);
            SpriteBatch.End();
#endif

            ScreenContext.SwapChain.Present(0, PresentFlags.None);
        }

        protected override void OnClientSizeChanged(EventArgs e)
        {
            base.OnClientSizeChanged(e);

#if VSG_DEBUG
#else
if(SpriteBatch!=null)SpriteBatch.Resize();
#endif
        }

        public override void Initialize(RenderContext context = null)
        {
            base.Initialize(context);

#if VSG_DEBUG
#else
SpriteBatch=new D2DSpriteBatch(RenderContext);
#endif
        }

        protected override void Dispose(bool disposing)
        {
            if (SpriteBatch != null) SpriteBatch.Dispose();
            base.Dispose(disposing);
        }
    }
}