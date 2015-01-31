using System;
using System.Drawing;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using Assimp;
using MMF.DeviceManager;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;
using Size = System.Drawing.Size;

namespace MMF.Controls.WPF
{
    /// <summary>
    ///     Interaction logic for WPFRenderControl.xaml
    /// </summary>
    public partial class WPFRenderControl : UserControl, IDisposable
    {
        public bool KeepAspectRatio = false;

        public WPFRenderControl()
        {
            InitializeComponent();
        }

        public WPFRenderControl(RenderContext context)
        {
            RenderContext = context;
            InitializeComponent();
        }

        /// <summary>
        ///     レンダーコンテキスト
        /// </summary>
        public RenderContext RenderContext { get; private set; }

        /// <summary>
        ///     テクスチャのレンダリングコンテキスト
        /// </summary>
        public TextureTargetContext TextureContext { get; private set; }

        public event EventHandler Render=delegate{};

        /// <summary>
        ///     このコントロールのワールド空間
        /// </summary>
        public WorldSpace WorldSpace
        {
            get { return TextureContext.WorldSpace; }
            set { TextureContext.WorldSpace = value; }
        }

        /// <summary>
        ///     背景色のクリア色
        /// </summary>
        public new Color4 Background
        {
            get { return TextureContext.BackgroundColor; }
            set { TextureContext.BackgroundColor = value; }
        }

        /// <summary>
        /// </summary>
        protected D3DImageContainer ImageContainer { get; private set; }

        public virtual void Dispose()
        {
        }

        protected virtual RenderContext getRenderContext()
        {
            RenderContext returnValue = RenderContext.CreateContext();
            return returnValue;
        }

        protected virtual TextureTargetContext GetTextureTargetContext()
        {
            //最初のサイズはおかしいので、とりあえず100,100を与えて、最初にリサイズイベントでリサイズする
            return new WPFTargetTextureContext(RenderContext, new Size(100, 100), new SampleDescription(1, 0));
        }

        protected override void OnInitialized(EventArgs e)
        {
            base.OnInitialized(e);
            if (RenderContext == null) RenderContext = getRenderContext();
            TextureContext = GetTextureTargetContext();
            //TextureContext.BackgroundColor=new Color4(1,0,1,1);
            RenderContext.UpdateRequireWorlds.Add(TextureContext.WorldSpace);
            ImageContainer = new D3DImageContainer();
            ImageContainer.IsFrontBufferAvailableChanged += ImageContainer_IsFrontBufferAvailableChanged;
            TargetImg.Source = ImageContainer;
            ImageContainer.SetBackBufferSlimDX(TextureContext.RenderTarget);
            Dispatcher.ShutdownStarted += Dispatcher_ShutdownStarted;
            BeginRenderingScene();
            //////テスト用コード
            //BasicGrid grid = new BasicGrid();
            //grid.Load(RenderContext);
            //TextureContext.WorldSpace.AddResource(grid);
            //TextureContext.MatrixManager.ViewMatrixManager.CameraPosition = new Vector3(0, 20, -40);
            //TextureContext.MatrixManager.ViewMatrixManager.CameraLookAt = new Vector3(0, 15, 0);
        }

        private void Dispatcher_ShutdownStarted(object sender, EventArgs e)
        {
            RenderContext.UpdateRequireWorlds.Remove(WorldSpace);

            if (ImageContainer != null)
            {
                ImageContainer.Dispose();
                ImageContainer = null;
            }
            if (TextureContext != null)
            {
                TextureContext.Dispose();
                TextureContext = null;
            }
            Dispose();
        }

        private void BeginRenderingScene()
        {
            if (ImageContainer.IsFrontBufferAvailable)
            {
                Texture2D texture = TextureContext.RenderTarget;
                ImageContainer.SetBackBufferSlimDX(texture);
                CompositionTarget.Rendering += Rendering;
            }
        }

        private void StopRenderingScene()
        {
            CompositionTarget.Rendering -= Rendering;
        }

        private void Rendering(object sender, EventArgs e)
        {
            if (TextureContext == null || ImageContainer == null) return;
            TextureContext.Render();
            ImageContainer.InvalidateD3DImage();
            Render(this,new EventArgs());
        }

        private void ImageContainer_IsFrontBufferAvailableChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            if (ImageContainer.IsFrontBufferAvailable)
            {
                BeginRenderingScene();
            }
            else
            {
                StopRenderingScene();
            }
        }

        private void WPFRenderControl_OnSizeChanged(object sender, SizeChangedEventArgs e)
        {
            if (TextureContext != null) TextureContext.Size = new Size((int) e.NewSize.Width, (int) e.NewSize.Height);
            if (ImageContainer != null && TextureContext != null)
                ImageContainer.SetBackBufferSlimDX(TextureContext.RenderTarget);
            if (!KeepAspectRatio && TextureContext != null)
                TextureContext.MatrixManager.ProjectionMatrixManager.AspectRatio =
                    (float) (e.NewSize.Width/e.NewSize.Height);
        }

        protected override void OnMouseDown(MouseButtonEventArgs e)
        {
            base.OnMouseDown(e);
            if(e.LeftButton==MouseButtonState.Pressed)TextureContext.HitChecker.IsMouseDown = true;
        }

        protected override void OnMouseUp(MouseButtonEventArgs e)
        {
            base.OnMouseUp(e);
            if (e.LeftButton == MouseButtonState.Released) TextureContext.HitChecker.IsMouseDown = false;
        }

        protected override void OnMouseMove(MouseEventArgs e)
        {
            base.OnMouseMove(e);
            var pos=e.GetPosition(this);
            TextureContext.HitChecker.CheckPoint=new System.Drawing.Point((int) pos.X,(int) pos.Y);
        }
    }
}