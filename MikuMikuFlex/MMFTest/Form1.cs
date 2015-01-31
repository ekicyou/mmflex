#define VSG_DEBUG
#region

using System;
using System.ComponentModel;
using System.Drawing;
using System.Windows.Forms;
using CGTest.Properties;
using MMF;
using MMF.Controls.Forms;
using MMF.DeviceManager;
using MMF.Grid;
using MMF.Kinect;
using MMF.Matricies.Camera.CameraMotion;
using MMF.Model.Controller;
using MMF.Model.Shape;
using MMF.Model.Shape.Overlay;
using MMF.Sprite;
using MMF.Sprite.D2D;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DirectWrite;
using SlimDX.DXGI;
using Brush = SlimDX.Direct2D.Brush;

#endregion

namespace CGTest
{
    public partial class Form1 : D2DSupportedRenderForm
    {
        public static TransformController Controller;

        public static ColorTexture ColTexture;
        // private Brush brush;
      

        private D2DSpriteTextformat format;

        private IDynamicTexture tex2;
        private ShaderResourceView resourceView;
        private float t = 0;
        public Form1()
        {
            InitializeComponent();
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            WorldSpace.AddDrawableGroup(new ControllerDrawableGroup(1,"Controller",RenderContext));
            KeyPreview = true;
            KinectDeviceManager dev = null;
            //BasicGrid grid=new BasicGrid();
            //grid.Load(RenderContext);
            Controller=new TransformController(RenderContext,ScreenContext.HitChekcer);
            WorldSpace.AddResource(Controller,"Controller");
            //if (!String.IsNullOrEmpty(Settings.Default.InitLoadModel))
            //{
            //    MMDModel model = MMDModelWithPhysics.OpenLoad(Settings.Default.InitLoadModel, RenderContext);
            //    WorldSpace.AddResource(model);
            //    if (!String.IsNullOrEmpty(Settings.Default.InitLoadMotion))
            //    {
            //        model.MotionManager.ApplyMotion(model.MotionManager.AddMotionFromFile(Settings.Default.InitLoadMotion, false));
            //    }
            //}
            //更新する必要のあるワールドスペースは、UpdateRequireWorldsに追加しなけれbなりません。
            //PlaneBoard bill = new PlaneBoard(RenderContext, resourceView, new Vector2(800, 800));
            //WorldSpace.AddResource(bill);
            //bill.Transformer.Position = new Vector3(0, 0, 20);
            #region Kinectテストコード

#if KINECT
            OpenNIManager.Initialize();
            dev = OpenNIManager.getDevice();

            ColTexture = new ColorTexture(RenderContext, dev);
            tex2 = new DepthTexture(RenderContext, 1000, dev);
#endif

            #endregion

            //format = SpriteBatch.CreateTextformat("Meiriyo", 30, FontWeight.ExtraBold);


            //brush = SpriteBatch.CreateRadialGradientBrush(g,
            //    new RadialGradientBrushProperties() {CenterPoint = new PointF(100,100),GradientOriginOffset = new PointF(0,0),HorizontalRadius = 100f,VerticalRadius = 200f}).Brush;
            //brush = SpriteBatch.CreateSolidColorBrush(Color.Aquamarine);
            ScreenContext.CameraMotionProvider = new BasicCameraControllerMotionProvider(this, this);
            BasicGrid gird = new BasicGrid();
            gird.Visibility = true;
            gird.Load(RenderContext);

            ScreenContext.WorldSpace.AddResource(gird);
            //textureの世界には、childウィンドウに追加からできるようにしてあります。
            ControlForm controlForm = new ControlForm(RenderContext, ScreenContext, null, dev);
            controlForm.Show(this);

            //OpenFileDialog ofd = new OpenFileDialog();
            //if (ofd.ShowDialog() == DialogResult.OK)
            //{
            //    VMDCameraMotionProvider camMotion=VMDCameraMotionProvider.OpenFile(ofd.FileName);
            //    ScreenContext.CameraMotionProvider = camMotion;
            //    camMotion.Start();
            //}
            

            #region Kinectテストコード
#if KINECT
            PlaneBoard bill = new PlaneBoard(RenderContext, ColTexture.TextureResourceView);
            bill.Transformer.Position = new Vector3(0, 0, 50);
            //bill.Transformer.Rotation *= Quaternion.RotationAxis(new Vector3(0,1,0),(float) (Math.PI));
            form.WorldSpace.AddResource(bill);
            form.WorldSpace.AddDynamicTexture(ColTexture);
#endif

            #endregion
        }

        protected override void OnClosing(CancelEventArgs e)
        {
            base.OnClosing(e);
            e.Cancel = false;
        }

        protected override void RenderSprite()
        {
            
        }

        public override void OnUpdated()
        {
            base.OnUpdated();
        }

        ///// <summary>
        /////     ここでスプライト描画
        ///// </summary>
        //protected override void RenderSprite()
        //{

        //    //SpriteBatch.DWRenderTarget.FillRectangle(brush,SpriteBatch.FillRectangle);
        //    //SpriteBatch.DWRenderTarget.DrawText(
        //    //    string.Format("FPS:{0}\n\nBrush Test!!", FpsCounter.FPS.ToString("####.#")), format,
        //    //    SpriteBatch.FillRectangle, brush);
        //}

        protected override void OnPresented()
        {
            base.OnPresented();
            RenderContext.LightManager.Position = new Vector3((float)Math.Cos(t),0, (float)Math.Sin(t)) * 10f;
            t += 0.0001f;
            //if(form.Visible)form.Render();
        }
    }
}