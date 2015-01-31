using System;
using System.Windows.Forms;
using MMF.Model.PMX;
using MMF.Motion;

namespace _09_Render3DCGToUserControl
{

    //②-A フォームデザイナを利用して画面をレイアウトする
    public partial class Form1 :Form
    {
        private PMXModel model;
        public Form1()
        {
            InitializeComponent();
        }

        /*
         * ②-B 利用しているRenderControlのInitializeメソッドを呼び出す
         */
        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            renderControl1.Initialize();
        }

        /*
         * ②-C レンダリング時にMessagePumpで呼び出すためのメソッドを定義しておく
         */
        public void Render()
        {
            //使用しているコントロールのRenderメソッドを呼び出すことでコントロールは描画を実行する。
            renderControl1.Render();
        }


        #region ②-D ボタンに応じて処理をする。今までのサンプルとそこまで内容は変わらない

        private void loadMotion_Click(object sender, EventArgs e)
        {
            if (model == null) return;
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Filter = "vmdモデルファイル(*.vmd)|*.vmd";
            if (ofd.ShowDialog(this) == DialogResult.OK)
            {
                IMotionProvider motion = model.MotionManager.AddMotionFromFile(ofd.FileName, false);
                model.MotionManager.ApplyMotion(motion);
            }
        }

        private void loadModel_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Filter = "pmxモデルファイル(*.pmx)|*.pmx";
            if (ofd.ShowDialog(this) == DialogResult.OK)
            {
                model = PMXModelWithPhysics.OpenLoad(ofd.FileName, renderControl1.RenderContext);
                renderControl1.WorldSpace.AddResource(model);
            }
        }

        #endregion

    }
}
