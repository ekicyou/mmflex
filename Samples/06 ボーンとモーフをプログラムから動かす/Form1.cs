using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MMF;
using MMF.Controls.Forms;
using MMF.Model.PMX;

namespace _06_MoveBoneAndMorphFromCode
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
                WorldSpace.AddResource(model);
                //②ボーン、モーフの編集用に作成したサンプルのGUIを表示する
                TransformController controller=new TransformController(model);
                controller.Show(this);
            }
        }
    }
}
