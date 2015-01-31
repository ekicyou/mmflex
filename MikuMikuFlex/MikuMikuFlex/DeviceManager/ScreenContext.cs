using System.Windows.Forms;
using MMF.Matricies;
using MMF.Matricies.Camera;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;
using Device = SlimDX.Direct3D11.Device;
using Resource = SlimDX.Direct3D11.Resource;

namespace MMF.DeviceManager
{
    public class ScreenContext:TargetContextBase
    {
        public ScreenContext(Control owner,RenderContext context,MatrixManager manager):base(context)
        {
            Device device = context.DeviceManager.Device;
            SampleDescription sampleDesc = new SampleDescription(1, 0);
            SwapChain = new SwapChain(context.DeviceManager.Factory, device, getSwapChainDescription(owner, sampleDesc));
            //ラスタライザの設定
            //深度ステンシルバッファの初期化
            using (Texture2D depthBuffer = new Texture2D(device, getDepthBufferTexture2DDescription(owner, sampleDesc)))
            {
                DepthTargetView = new DepthStencilView(device, depthBuffer);
            }
            //レンダーターゲットの初期化
            using (Texture2D renderTexture = Resource.FromSwapChain<Texture2D>(SwapChain, 0))
            {
                RenderTargetView = new RenderTargetView(device, renderTexture);
            }
            WorldSpace=new WorldSpace(context);
            BindedControl = owner;
            MatrixManager = manager;
            PanelObserver=new PanelObserver(owner);
            SetViewport();
            HitChekcer=new TexturedBufferHitChecker(Context,this);
            HitChekcer.Resize(owner.ClientSize);
            owner.MouseMove += owner_MouseMove;
            owner.MouseDown += owner_MouseDown;
            owner.MouseUp += owner_MouseUp;
        }

        void owner_MouseUp(object sender, MouseEventArgs e)
        {
            if(e.Button==MouseButtons.Left)
            HitChekcer.IsMouseDown = false;
        }

        void owner_MouseDown(object sender, MouseEventArgs e)
        {
            if(e.Button==MouseButtons.Left)
            HitChekcer.IsMouseDown = true;
        }

        void owner_MouseMove(object sender, MouseEventArgs e)
        {
            HitChekcer.CheckPoint = e.Location;
        }

        public TexturedBufferHitChecker HitChekcer;

        /// <summary>
        /// Gets or sets the matrix manager.
        /// </summary>
        /// <value>
        /// The matrix manager.
        /// </value>
        public override MatrixManager MatrixManager { get; set; }

        /// <summary>
        /// Gets the swap chain.
        /// </summary>
        /// <value>
        /// The swap chain.
        /// </value>
        public SwapChain SwapChain { get; private set; }

        /// <summary>
        /// Gets the binded control.
        /// </summary>
        /// <value>
        /// The binded control.
        /// </value>
        public Control BindedControl { get; private set; }


        /// <summary>
        /// カメラモーションの挙動を設定するプロパティ
        /// </summary>
        public override ICameraMotionProvider CameraMotionProvider { get; set; }


        /// <summary>
        /// このスクリーンに結び付けられているワールド空間
        /// </summary>
        public override WorldSpace WorldSpace { get; set; }

        /// <summary>
        /// ユーザーコントロールを監視するクラス
        /// </summary>
        public PanelObserver PanelObserver { get; private set; }


        public override void SetViewport()
        {
            Context.DeviceManager.Context.Rasterizer.SetViewports(getViewport());
        }

        public void SetPanelObserver()
        {
            Context.CurrentPanelObserver = PanelObserver;
        }


        public void Resize()
        {
            if (BindedControl.ClientSize.Width == 0 || BindedControl.ClientSize.Height == 0) return; //フォームがフロート状態になった時一瞬だけ来て、デバイスが作れなくなるのでこの時はなしにする。
            DisposeTargetViews();
            SwapChainDescription desc = SwapChain.Description;
            SwapChain.ResizeBuffers(desc.BufferCount, BindedControl.ClientSize.Width, BindedControl.ClientSize.Height, desc.ModeDescription.Format,
                desc.Flags);
            //深度ステンシルバッファの初期化
            using (Texture2D depthBuffer = new Texture2D(Context.DeviceManager.Device, getDepthBufferTexture2DDescription(BindedControl, desc.SampleDescription)))
            {
                DepthTargetView = new DepthStencilView(Context.DeviceManager.Device, depthBuffer);
            }
            //レンダーターゲットの初期化
            using (Texture2D renderTexture = Resource.FromSwapChain<Texture2D>(SwapChain, 0))
            {
                RenderTargetView = new RenderTargetView(Context.DeviceManager.Device, renderTexture);
            }
            HitChekcer.Resize(BindedControl.ClientSize);
        }

        public void MoveCameraByCameraMotionProvider()
        {
            if(CameraMotionProvider==null)return;
            CameraMotionProvider.UpdateCamera(MatrixManager.ViewMatrixManager,MatrixManager.ProjectionMatrixManager);
        }

        /// <summary>
        ///     スワップチェーンの設定を取得します。
        ///     スワップチェーンの設定を変えたい場合は、オーバーライドしてください。
        /// </summary>
        /// <param name="control">適応するコントロールへの参照</param>
        /// <returns>スワップチェーンの設定</returns>
        protected virtual SwapChainDescription getSwapChainDescription(Control control, SampleDescription sampDesc)
        {
            return new SwapChainDescription
            {
                BufferCount = 2,
                Flags = SwapChainFlags.AllowModeSwitch,
                IsWindowed = true,
                ModeDescription = new ModeDescription
                {
                    Format = Format.R8G8B8A8_UNorm,
                    Height = control.ClientSize.Height,
                    Width = control.ClientSize.Width,
                    RefreshRate = new Rational(60, 1)
                },
                OutputHandle = control.Handle,
                SampleDescription = sampDesc,
                SwapEffect = SwapEffect.Discard,
                Usage = Usage.RenderTargetOutput
            };
        }

        /// <summary>
        ///     深度ステンシルバッファの設定を取得します。
        ///     深度ステンシルバッファの設定を変えたい場合はオーバーライドしてください。
        /// </summary>
        /// <param name="control">適用するコントロールへの参照</param>
        /// <returns>深度ステンシルバッファ用のTexture2Dの設定</returns>
        protected virtual Texture2DDescription getDepthBufferTexture2DDescription(Control control, SampleDescription desc)
        {
            return new Texture2DDescription
            {
                ArraySize = 1,
                BindFlags = BindFlags.DepthStencil,
                Format = Format.D32_Float,
                Width = control.ClientSize.Width,
                Height = control.ClientSize.Height,
                MipLevels = 1,
                SampleDescription = desc
            };
        }

        /// <summary>
        ///     ビューポートの内容を取得します。
        /// </summary>
        /// <param name="control">適用するコントロールへの参照</param>
        /// <returns>設定するビューポート</returns>
        protected virtual Viewport getViewport()
        {
            return new Viewport
            {
                Width = BindedControl.Width,
                Height = BindedControl.Height,
                MaxZ = 1
            };
        }

        public override void Dispose()
        {
            if (WorldSpace!=null)
            {
                WorldSpace.Dispose();
            }
            if (RenderTargetView != null && !RenderTargetView.Disposed) RenderTargetView.Dispose();
            if (DepthTargetView != null && !DepthTargetView.Disposed) DepthTargetView.Dispose();
            if (SwapChain != null && !SwapChain.Disposed) SwapChain.Dispose();
            HitChekcer.Dispose();
        }
    }
}
