using System;
using System.Windows.Forms;
using MMF;
using MMF.Controls.Forms;
using MMF.Model.PMX;
using MMF.Motion;

namespace _05_HowToUpdateCamera
{
    public partial class Form1 : RenderForm
    {
        public Form1()
        {
            InitializeComponent();
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);

            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Filter = "pmxモデルファイル(*.pmx)|*.pmx";
            if (ofd.ShowDialog() == DialogResult.OK)
            {
                PMXModel model = PMXModelWithPhysics.OpenLoad(ofd.FileName, RenderContext);

                OpenFileDialog ofd2 = new OpenFileDialog();
                ofd2.Filter = "vmdモーションファイル(*.vmd)|*.vmd";
                if (ofd2.ShowDialog() == DialogResult.OK)
                {
                    IMotionProvider motion = model.MotionManager.AddMotionFromFile(ofd2.FileName, true);
                    model.MotionManager.ApplyMotion(motion, 0, ActionAfterMotion.Replay);
                }
                WorldSpace.AddResource(model);
                //③ カメラモーションの選択ダイアログを表示し、選ばれたものをScreenContext.CameraMotionProviderに代入する。
                CameraControlSelector selector=new CameraControlSelector(model);
                selector.ShowDialog(this);
                ScreenContext.CameraMotionProvider = selector.ResultCameraMotionProvider;
                /*
                 * ScreenContext.CameraMotionProviderに代入されたインターフェースのUpdateCameraが毎回呼ばれることによりカメラを更新している。
                 * この変数の型はICameraMotionProviderのため、これを実装すればカメラの動きは容易に定義可能である。
                 */
            }
        }
    }
}
