using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using CGTest.Properties;
using MMF;
using MMF.Bone;
using MMF.DeviceManager;
using MMF.Kinect;
using MMF.Model.Assimp;
using MMF.Model.Controller;
using MMF.Model.PMX;
using MMF.Motion;
using SlimDX;

namespace CGTest
{
    public partial class ControlForm : Form
    {

        public RenderContext Context { get; private set; }

        public PMXModel Model { get; private set; }

        public IMotionProvider CurrentMotion { get; private set; }

        public bool isPlaying { get; private set; }

        private KinectFKUpdater updater;

        private readonly ScreenContext scContext;
        private readonly ITargetContext _sccContext;
        private KinectDeviceManager device;

        public ControlForm(RenderContext context,ScreenContext scContext,ITargetContext sccContext,KinectDeviceManager device)
        {
            Context = context;
            this.scContext = scContext;
            _sccContext = sccContext;
            this.device = device;
            InitializeComponent();
            Model_Load.Click += Model_Load_Click;
            Motion_Load.Click += Motion_Load_Click;
            frameSelector.ValueChanged += frameSelector_ValueChanged;
            play.Click += play_Click;
            stop.Click += stop_Click;
            UpdatePositionData();
        }

        void stop_Click(object sender, EventArgs e)
        {
            if (CurrentMotion != null)
            {
                CurrentMotion.Stop();
                isPlaying = false;
            }
        }

        void play_Click(object sender, EventArgs e)
        {
            if (CurrentMotion != null)
            {
                CurrentMotion.Start(CurrentMotion.CurrentFrame, ActionAfterMotion.Replay);
                isPlaying = true;
            }
        }

        void frameSelector_ValueChanged(object sender, EventArgs e)
        {
            if (!isPlaying&&CurrentMotion!=null)
            {
                CurrentMotion.CurrentFrame = frameSelector.Value;
                frameLabel.Text = string.Format("{0}フレーム中、{1}フレーム目", CurrentMotion.FinalFrame,
                CurrentMotion.CurrentFrame.ToString("#.0"));

            }
        }

        void Motion_Load_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Title = "開く対象のVMD/VMEファイル";
            ofd.Filter = "VMDモーションファイル(*.vmd)|*.vmd|VMEモーションファイル(*.vme)|*.vme";
            if (ofd.ShowDialog() == DialogResult.OK)
            {
                CurrentMotion = Model.MotionManager.AddMotionFromFile(ofd.FileName,false);
                Model.MotionManager.ApplyMotion(CurrentMotion,0,ActionAfterMotion.Replay);
                frameSelector.Maximum = CurrentMotion.FinalFrame;
                frameSelector.Minimum = 0;
                CurrentMotion.FrameTicked += CurrentMotion_FrameTicked;
                Settings.Default.InitLoadMotion = ofd.FileName;

                #region VMDカメラモーションテストコード

                //VMDCameraMotionProvider provider = new VMDCameraMotionProvider(MotionData.getMotion(File.OpenRead(@"C:\Users\Lime\Desktop\ハレ晴レユカイ\camera.vmd")));
                //Context.CameraMotionProvider = provider;
                //provider.Start();

                #endregion
            }
        }

        void CurrentMotion_FrameTicked(object sender, EventArgs e)
        {
            if (CurrentMotion == null) return;
            Invoke((MethodInvoker)delegate 
            {
                if(frameLabel.IsDisposed||frameSelector.IsDisposed)return;
                frameLabel.Text = string.Format("{0}フレーム中、{1}フレーム目", CurrentMotion.FinalFrame,
                    CurrentMotion.CurrentFrame.ToString("#.0"));
                frameSelector.Value = (int) CurrentMotion.CurrentFrame;
            });
        }

        void Model_Load_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofd=new OpenFileDialog();
            ofd.Title = "開く対象のPMXファイル";
            ofd.Filter = "PMXモデルファイル(*.pmx)|*.pmx";
            if (ofd.ShowDialog() == DialogResult.OK)
            {
                if (Model != null)
                {
                    scContext.WorldSpace.RemoveResource(Model);
                }
                Model = PMXModelWithPhysics.OpenLoad(ofd.FileName, Context);
                Model.Transformer.Position = new Vector3(0, 0, 0);
                #region KinectFKテストコード
#if KINECT
                updater = new KinectFKUpdater(device);
                updater.TrackingUser += updater_TrackingUser;
                Model.Skinning.KinematicsProviders[0] = updater;
                Model.Transformer.Position += new Vector3(0, 0, -30);
#endif
                #endregion
                scContext.WorldSpace.AddResource(Model);
                Form1.Controller.setTargetModel(Model);
                Form1.Controller.Type = TransformController.TransformType.All;
                Motion_Load.Enabled = true;
                isPlaying = true;
                Settings.Default.InitLoadModel = ofd.FileName;
            }
        }


        private void ControlForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            e.Cancel = true;
        }
        #region KinectFKテストコード
        private void updater_TrackingUser(object sender, NiTEWrapper.UserData e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
#if KINECT
            device.UserCursor++;
#endif
            }

        private void doTrack_Click(object sender, EventArgs e)
        {
#if KINECT
            updater.StartTracking(device.UserCursor);
#endif
            }
        #endregion

        private void reload_effect_Click(object sender, EventArgs e)
        {
            if (Model != null)
            {
                Model.LoadEffect("");
            }
        }

        private readonly float light_transvalue = 1.0f;

        private void trans_x_plus_Click(object sender, EventArgs e)
        {
            Context.LightManager.Position += new Vector3(1, 0, 0)*light_transvalue;
            UpdatePositionData();
        }

        private void trans_y_plus_Click(object sender, EventArgs e)
        {
            Context.LightManager.Position += new Vector3(0, 1, 0) * light_transvalue;
            UpdatePositionData();
        }

        private void trans_z_plus_Click(object sender, EventArgs e)
        {
            Context.LightManager.Position += new Vector3(0, 0, 1) * light_transvalue;
            UpdatePositionData();
        }

        private void trans_x_minus_Click(object sender, EventArgs e)
        {
            Context.LightManager.Position += new Vector3(-1, 0, 0) * light_transvalue;
            UpdatePositionData();
        }

        private void trans_y_minus_Click(object sender, EventArgs e)
        {
            Context.LightManager.Position += new Vector3(0, -1, 0) * light_transvalue;
            UpdatePositionData();
        }

        private void trans_z_minus_Click(object sender, EventArgs e)
        {
            Context.LightManager.Position += new Vector3(0, 0, -1) * light_transvalue;
            UpdatePositionData();
        }

        private void UpdatePositionData()
        {
            Vector3 position = Context.LightManager.Position;
            light_position.Text = string.Format("POSITION:{0},DIRECTION:{1}",position.ToString(),Context.LightManager.Direction);
        }

        private void Add2Child_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Title = "開く対象のPMXファイル";
            ofd.Filter = "PMXモデルファイル(*.pmx)|*.pmx";
            if (ofd.ShowDialog() == DialogResult.OK)
            {
                if (Model != null)
                {
                    _sccContext.WorldSpace.RemoveResource(Model);
                }
                Model = PMXModelWithPhysics.OpenLoad(ofd.FileName, Context);
                Model.Transformer.Position = new Vector3(0, 0, 0);
                #region KinectFKテストコード
#if KINECT
                updater = new KinectFKUpdater(device);
                updater.TrackingUser += updater_TrackingUser;
                Model.Skinning.KinematicsProviders[0] = updater;
                Model.Transformer.Position += new Vector3(0, 0, -30);
#endif
                #endregion

                _sccContext.WorldSpace.AddResource(Model);
                Motion_Load.Enabled = true;
                isPlaying = true;


            }
        }

        private void AddAssimpButton_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofd=new OpenFileDialog();
            if (ofd.ShowDialog() == DialogResult.OK)
            {
                AssimpModel assimp=new AssimpModel(Context,ofd.FileName);
                scContext.WorldSpace.AddResource(assimp);
            }
        }
    }
}
