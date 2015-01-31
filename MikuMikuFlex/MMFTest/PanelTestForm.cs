using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MMF.Grid;
using MMF.Matricies.Camera.CameraMotion;
using SlimDX;

namespace CGTest
{
    public partial class PanelTestForm : Form
    {
        public PanelTestForm()
        {
            InitializeComponent();
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            leftTop.Initialize();
            BasicGrid grid=new BasicGrid();
            grid.Load(leftTop.RenderContext);
            leftTop.WorldSpace.AddResource(grid);
            rightTop.Initialize(leftTop.RenderContext);
            rightBottom.Initialize(leftTop.RenderContext);
            leftBottom.Initialize(leftTop.RenderContext);
            rightTop.WorldSpace = leftTop.WorldSpace;
            rightBottom.WorldSpace = leftTop.WorldSpace;
            leftBottom.WorldSpace = leftTop.WorldSpace;
            leftTop.ScreenContext.CameraMotionProvider=new BasicCameraControllerMotionProvider(leftTop,this);
            rightTop.ScreenContext.CameraMotionProvider=new SideCameraMotionProvider(leftTop.ScreenContext.CameraMotionProvider,Quaternion.RotationAxis(new Vector3(0,1,0),(float) (Math.PI) ));
            leftBottom.ScreenContext.CameraMotionProvider = new SideCameraMotionProvider(leftTop.ScreenContext.CameraMotionProvider, Quaternion.RotationAxis(new Vector3(0, 1, 0), (float)(Math.PI / 2)));
            rightBottom.ScreenContext.CameraMotionProvider = new SideCameraMotionProvider(leftTop.ScreenContext.CameraMotionProvider, Quaternion.RotationAxis(new Vector3(0, 1, 0), -(float)(Math.PI / 2)));
            ControlForm form=new ControlForm(leftTop.RenderContext,leftTop.ScreenContext,leftTop.ScreenContext,null);
            form.Show();
        }

        public void Render()
        {
            leftTop.Render();
            rightTop.Render();
            rightBottom.Render();
            leftBottom.Render();
        }
    }
}
