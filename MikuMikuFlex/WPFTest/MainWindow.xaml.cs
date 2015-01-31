using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using MMF;
using MMF.CG;
using MMF.CG.DeviceManager;
using MMF.CG.Model.Grid;
using MMF.CG.Model.Shape;
using MMF.WPF;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.Direct3D9;
using SlimDX.DXGI;
using Size=System.Drawing.Size;

namespace WPFTest
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private readonly Form1 form;
        private D3DImageContainer container;
        private TextureTargetContext context;
        private RenderContext renderContext;

        public MainWindow()
        {
            //this.form = form;
            InitializeComponent();
        }

        //private void MainWindow_OnLoaded(object sender, RoutedEventArgs e)
        //{
        //    renderContext =new RenderContext();
        //    renderContext.Initialize();
        //    context = new WPFTargetTextureContext(renderContext,new Size(800,800),new SampleDescription(1,0));
        //    context.BackgroundColor = new Color4(1, 0, 1, 1);
        //    PlaneBoard pb=new PlaneBoard(renderContext,new ShaderResourceView(renderContext.DeviceManager.Device,context.RenderTarget));
        //    BasicGrid grid=new BasicGrid();
        //    grid.Load(renderContext);
        //    context.WorldSpace.AddResource(grid);
        //    container=new D3DImageContainer();
        //    container.IsFrontBufferAvailableChanged += container_IsFrontBufferAvailableChanged;
        //    SlimDXImage.Source = container;
        //    container.SetBackBufferSlimDX(context.RenderTarget);
        //    BeginRenderingScene();
        //}

        //protected override void OnClosed(EventArgs e)
        //{
        //    base.OnClosed(e);
        //    if (container != null)
        //    {
        //        container.Dispose();
        //    }
        //    if (context != null)
        //    {
        //        context.Dispose();
        //    }
        //    renderContext.Dispose();
        //}

        //private void container_IsFrontBufferAvailableChanged(object sender, DependencyPropertyChangedEventArgs e)
        //{
        //    if (container.IsFrontBufferAvailable)
        //    {
        //        BeginRenderingScene();
        //    }
        //    else
        //    {
        //        StopRenderingScene();
        //    }
        //}

        //protected override void OnRender(DrawingContext drawingContext)
        //{
        //    base.OnRender(drawingContext);
        //    Render();
        //}

        //private void StopRenderingScene()
        //{
        //    CompositionTarget.Rendering -= OnRendering;
        //}

        //private void BeginRenderingScene()
        //{
        //    if (container.IsFrontBufferAvailable)
        //    {
        //        Texture2D texture = context.RenderTarget;
        //        container.SetBackBufferSlimDX(texture);
        //        CompositionTarget.Rendering += OnRendering;
        //    }
        //}

        //public void Render()
        //{
        //    if(context==null||container==null)return;
        //    context.Render();
        //    container.InvalidateD3DImage();
        //}

        //private void OnRendering(object sender, EventArgs e)
        //{
        //    Render();
        //}
    }
}
