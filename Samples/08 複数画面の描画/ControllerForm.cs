using System;
using System.Windows.Forms;
using MMF.Model.PMX;

namespace _08_MultiScreenRendering
{
    public partial class ControllerForm : Form
    {
        private readonly Form1 form1;
        private readonly ChildForm childForm;

        public ControllerForm(Form1 form1,ChildForm childForm)
        {
            this.form1 = form1;
            this.childForm = childForm;
            InitializeComponent();
        }

        #region ②-A ボタンに応じたワールド空間に対してモデルを追加する

        private void Add2ChildForm_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Filter = "pmxモデルファイル(*.pmx)|*.pmx";
            if (ofd.ShowDialog() == DialogResult.OK)
            {
                PMXModel model = PMXModelWithPhysics.OpenLoad(ofd.FileName, childForm.RenderContext);
                    //ここで渡すRenderContextはform1とchildFormで共通しているのでどちらでも良い
                childForm.WorldSpace.AddResource(model);
            }
        }

        private void Add2Form1_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Filter = "pmxモデルファイル(*.pmx)|*.pmx";
            if (ofd.ShowDialog() == DialogResult.OK)
            {
                PMXModel model = PMXModelWithPhysics.OpenLoad(ofd.FileName, form1.RenderContext);
                form1.WorldSpace.AddResource(model);
            }
        }

        #endregion

    }
}
