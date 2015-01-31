using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MMF;
using MMF.CG.Model.MMD;
using MMF.CG.Motion;

namespace _03_ApplyMotionToPMX
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
            ofd.Filter = "pmx model file(*.pmx)|*.pmx";
            if (ofd.ShowDialog() == DialogResult.OK)
            {
                MMDModel model = MMDModelWithPhysics.OpenLoad(ofd.FileName, RenderContext);

                //OpenFileDialog to select vmd motion file
                OpenFileDialog ofd2=new OpenFileDialog();
                ofd2.Filter = "vmd motion file(*.vmd)|*.vmd";
                if (ofd2.ShowDialog() == DialogResult.OK)
                {
                    //①Load motion file
                    IMotionProvider motion = model.MotionManager.AddMotionFromFile(ofd2.FileName, true);
                    //You should add your motion file to the model you wanting to apply.
                    //IMotionProvider AddMotionFromFile(string fileName,bool ignoreParentBone);
                    //If you set true in secound argument to the method above,MMF will ignore root bones motion.
                    //For instance,you want to walk by your code and motion with motion file,motion file might move model coordinate.
                    //When MMF ignore parent bone,MMF will not move model coordinate by motion.

                    //②Apply motion to your PMX model
                    model.MotionManager.ApplyMotion(motion,0,ActionAfterMotion.Replay);
                    //Secound argument:
                    //Start frame number
                    //Third argument:
                    //What MMF should do when motion playing is finished.
                    //If you want MMF to do nothing when motion playing is finished,you should set ActionAfterMotion.Nothing in third argument.

                    //Extra
                    //(1) How to stop motion?
                    //You can stop motion by using the code below.
                    //model.MotionManager.StopMotion();
                    //(2) What frame number is it playing?
                    //You can get form model.MotionManager.CurrentFrame.
                }
                WorldSpace.AddResource(model);
            }
        }
    }
}
