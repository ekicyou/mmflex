using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MMF;
using MMF.Controls.Forms;
using MMF.Model.PMX;
using MMF.Motion;

namespace _04_TransformModelFormCode
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
                //②コントローラーフォームに対して読み込んだモデルを渡して表示します。
                Controller controller=new Controller(model);
                controller.Show();
            }
        }
    }
}
