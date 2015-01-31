using System;
using System.IO;
using System.Windows;
using Microsoft.Win32;
using MMF.Matricies.Camera.CameraMotion;
using MMF.Model.PMX;
using SlimDX;

namespace Sample10._5
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        protected override void OnInitialized(EventArgs e)
        {
            base.OnInitialized(e);
            OpenFileDialog ofd=new OpenFileDialog();
            ofd.Title = "表示するモデルファイルを選んでください。";
            ofd.Filter = "PMXモデルファイル(*.pmx)|*.pmx";
            if (ofd.ShowDialog() == true)
            {
                PMXModel model = PMXModel.OpenLoad(ofd.FileName, RenderControl.RenderContext);
                RenderControl.WorldSpace.AddResource(model);
                RenderControl.TextureContext.MatrixManager.ViewMatrixManager.CameraPosition = new Vector3(0, 10, -25);
                RenderControl.TextureContext.MatrixManager.ViewMatrixManager.CameraLookAt = new Vector3(0, 10, 0);
                RenderControl.TextureContext.CameraMotionProvider=new WPFBasicCameraControllerMotionProvider(RenderControl,10);//コントロールを完全に透過すると、色のある部分の上しか反応しないので、Backgroundを#01000000などにすることでウィンドウ全域で反応するようになる。
            }
        }
    }
}
