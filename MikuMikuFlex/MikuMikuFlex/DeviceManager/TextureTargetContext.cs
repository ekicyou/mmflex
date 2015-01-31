using System;
using System.Drawing;
using System.Windows.Forms;
using MMF.Matricies;
using MMF.Matricies.Camera;
using MMF.Matricies.Projection;
using MMF.Matricies.World;
using MMF.Motion;
using MMF.Utility;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;
using Device = SlimDX.Direct3D11.Device;

namespace MMF.DeviceManager
{
    public class TextureTargetContext : TargetContextBase
    {
        private bool disposed = false;

        private FPSCounter fpsCounter;

        private SampleDescription sampleDesc = new SampleDescription(1, 0);
        /// <summary>
        /// マルチサンプルの設定を取得または設定します．
        /// 可能な限り，指定された値に近い設定を採用します．
        /// </summary>
        public SampleDescription SampleDesc
        {
            get { return sampleDesc; }
            set
            {
                var device = Context.DeviceManager.Device;
                Format format = getRenderTargetTexture2DDescription().Format;
                int count = value.Count;
                do
                {
                    int msql = device.CheckMultisampleQualityLevels(format, count);
                    if (msql > 0)
                    {
                        int quality = Math.Min(msql - 1, value.Quality);
                        this.sampleDesc = new SampleDescription(count, quality);
                        break;
                    }

                    // マルチサンプル数がサポートされない場合
                    count--;
                } while (count > 0);
                if(size.Width>0&&size.Height>0)
                ResetTargets();
            }
        }

        private Size size;
        /// <summary>
        /// レンダーターゲットのサイズを取得または設定します．
        /// </summary>
        public Size Size
        {
            get { return size; }
            set
            {
                if (size != value&&value.Width>0&&value.Height>0)
                {
                    
                    size = value;
                    ResetTargets();
                }
            }
        }

        /// <summary>
        /// レンダーターゲットをリセットします．
        /// </summary>
        private void ResetTargets()
        {
            // ターゲットを破棄
            DisposeTargetViews();
            if(depthTarget!=null&&!depthTarget.Disposed)depthTarget.Dispose();
            if(renderTarget!=null&&!renderTarget.Disposed)renderTarget.Dispose();
            Device device = Context.DeviceManager.Device;
            //深度ステンシルバッファの初期化
            //レンダーターゲットの初期化
            this.renderTarget = new Texture2D(device, getRenderTargetTexture2DDescription());
            this.RenderTargetView = new RenderTargetView(device, RenderTarget);
            //深度ステンシルバッファの初期化
            this.depthTarget = new Texture2D(device, getDepthBufferTexture2DDescription());
            this.DepthTargetView = new DepthStencilView(device, DepthTarget);
            HitChecker.Resize(Size);
            SetViewport();
        }

        private Texture2D renderTarget;
        /// <summary>
        /// レンダー ターゲットを取得します．
        /// </summary>
        public Texture2D RenderTarget
        {
            get { return renderTarget; }
        }


        private Texture2D depthTarget;
        public TexturedBufferHitChecker HitChecker;

        /// <summary>
        /// Gets the depth target.
        /// </summary>
        public Texture2D DepthTarget
        {
            get { return depthTarget; }
        }

        public TextureTargetContext(RenderContext context, Size size, SampleDescription sampleDesc)
            : this(context,
            new MatrixManager(new BasicWorldMatrixProvider(), new BasicCamera(new Vector3(0, 20, -200), new Vector3(0, 3, 0), new Vector3(0, 1, 0)), new BasicProjectionMatrixProvider())
            , size, sampleDesc)
        {
            this.MatrixManager.ProjectionMatrixManager.InitializeProjection((float)Math.PI / 4f, (float)size.Width / size.Height, 1, 2000);
        }

        public TextureTargetContext(RenderContext context, MatrixManager matrixManager, Size size, SampleDescription sampleDesc):base(context)
        {
            HitChecker = new TexturedBufferHitChecker(context, this);
            context.Timer=new MotionTimer(context);
            Device device = context.DeviceManager.Device;

            // サイズを設定（ターゲットは初期化しない）
            this.size = size;
            // マルチサンプルの設定（ターゲットも初期化する）
            this.SampleDesc = sampleDesc;

            // その他設定
            this.MatrixManager = matrixManager;
            this.WorldSpace = new WorldSpace(context);
            fpsCounter=new FPSCounter();
            SetViewport();
            HitChecker=new TexturedBufferHitChecker(Context,this);
        }
        ~TextureTargetContext()
        {
            Dispose(false);
        }
        public override void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }
        protected void Dispose(bool disposing)
        {
            if (!disposed)
            {
                if (disposing)
                {
                    this.Context = null;
                }

                if (WorldSpace != null)
                {
                    WorldSpace.Dispose();
                    WorldSpace = null;
                }

                DisposeTargetViews();
                if (RenderTarget != null && !RenderTarget.Disposed)
                    RenderTarget.Dispose();
                if (DepthTarget != null && !DepthTarget.Disposed)
                    DepthTarget.Dispose();
                HitChecker.Dispose();
                disposed = true;
            }
        }

        ///// <summary>
        ///// レンダーターゲットの設定を取得します．
        ///// </summary>
        protected virtual Texture2DDescription getRenderTargetTexture2DDescription()
        {
            return new Texture2DDescription
            {
                Width = this.Size.Width,
                Height = this.Size.Height,
                MipLevels = 1,
                ArraySize = 1,
                Format = Format.B8G8R8A8_UNorm,
                SampleDescription = this.SampleDesc,
                Usage = ResourceUsage.Default,
                BindFlags = BindFlags.RenderTarget | BindFlags.ShaderResource,
                CpuAccessFlags = CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.None
            };
        }

        /// <summary>
        /// 深度ステンシルバッファの設定を取得します。
        /// </summary>
        protected virtual Texture2DDescription getDepthBufferTexture2DDescription()
        {
            return new Texture2DDescription
            {
                ArraySize = 1,
                BindFlags = BindFlags.DepthStencil,
                Format = Format.D32_Float,
                Width = this.Size.Width,
                Height = this.Size.Height,
                MipLevels = 1,
                SampleDescription = this.SampleDesc
            };
        }

        /// <summary>
        /// Gets or sets the matrix manager.
        /// </summary>
        /// <value>
        /// The matrix manager.
        /// </value>
        public override MatrixManager MatrixManager { get; set; }

        /// <summary>
        /// カメラモーションの挙動を設定するプロパティ
        /// </summary>
        public override ICameraMotionProvider CameraMotionProvider { get; set; }

        /// <summary>
        /// このスクリーンに結び付けられているワールド空間
        /// </summary>
        public override WorldSpace WorldSpace { get; set; }

        public override void SetViewport()
        {
            Context.DeviceManager.Context.Rasterizer.SetViewports(getViewport());
        }

        public void MoveCameraByCameraMotionProvider()
        {
            if (CameraMotionProvider == null)
                return;

            CameraMotionProvider.UpdateCamera(MatrixManager.ViewMatrixManager, MatrixManager.ProjectionMatrixManager);
        }

        /// <summary>
        /// ビューポートの内容を取得します。
        /// </summary>
        /// <returns>設定するビューポート</returns>
        protected virtual Viewport getViewport()
        {
            return new Viewport
            {
                Width = this.Size.Width,
                Height = this.Size.Height,
                MaxZ = 1
            };
        }

        /// <summary>
        /// 背景色を取得または設定します．
        /// </summary>
        public Color4 BackgroundColor { get; set; }

        /// <summary>
        /// WorldSpaceの内容を描画します．
        /// </summary>
        public void Render()
        {
            if(WorldSpace==null||WorldSpace.IsDisposed)return;
            Context.SetRenderScreen(this);
            Context.ClearScreenTarget(BackgroundColor);
            Context.Timer.TickUpdater();
            fpsCounter.CountFrame();
            MoveCameraByCameraMotionProvider();
            WorldSpace.DrawAllResources(HitChecker);
            Context.DeviceManager.Context.Flush();

        }
    }
}