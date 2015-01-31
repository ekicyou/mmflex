using System;
using System.Collections.Generic;
using System.Text;
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
using Microsoft.Win32;
using MMF.Grid;
using MMF.Matricies.Camera.CameraMotion;
using MMF.Model.PMX;
using SlimDX;

namespace _10_MMFForWPF1
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

            //② 使い方は今までのサンプルとほとんど同じだが、Renderメソッドなどを呼び出す必要性はない。
            //また、RenderContextは自動的に作成されるが、これが嫌な場合はWPFRenderControlをオーバーライドする必要がある。
            //初期化などはOnInitializedで行うことがおすすめ
            RenderControl.Background = new Color4(1, 0, 0, 1);
            BasicGrid grid=new BasicGrid();
            grid.Load(RenderControl.RenderContext);
            RenderControl.WorldSpace.AddResource(grid);
            RenderControl.TextureContext.CameraMotionProvider=new WPFBasicCameraControllerMotionProvider(this);
            OpenFileDialog ofd=new OpenFileDialog();
            if (ofd.ShowDialog() == true)
            {
                PMXModel model = PMXModelWithPhysics.OpenLoad(ofd.FileName, RenderControl.RenderContext);
                RenderControl.WorldSpace.AddResource(model);
            }
        }
    }
}
