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

namespace _03_ApplyVMDToPMX
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

                //モーションファイルの読み込みようダイアログの設置
                OpenFileDialog ofd2=new OpenFileDialog();
                ofd2.Filter = "vmdモーションファイル(*.vmd)|*.vmd";
                if (ofd2.ShowDialog() == DialogResult.OK)
                {
                    //ダイアログの返値がOKの場合、モデルの読み込み処理をする

                    //①モーションファイルを読み込む
                    IMotionProvider motion = model.MotionManager.AddMotionFromFile(ofd2.FileName, true);
                    //適用したい対象のモデルのモーションマネージャに対して追加します。
                    //IMotionProvider AddMotionFromFile(string ファイル名,bool すべての親ボーンを無視するかどうか);
                    //第二引数は歩きモーションなどで、移動自体はプログラムで指定したいとき、すべての親ボーンのモーションを無視することで、
                    //モーションでモデル全体が動いてしまうのを防ぎます。

                    //②モーションファイルをモデルに対して適用する。
                    model.MotionManager.ApplyMotion(motion,0,ActionAfterMotion.Replay);
                    //第二引数は、再生を始めるフレーム番号、第三引数は再生後にリプレイするかどうか。
                    //リプレイせず放置する場合はActionAfterMotion.Nothingを指定する

                    //ｵﾏｹ
                    //(1) モーションをとめるときは?
                    //model.MotionManager.StopMotion();と記述すれば止まります
                    //(2) 現在何フレーム目なの?
                    //model.MotionManager.CurrentFrameによって取得できます。
                }
                WorldSpace.AddResource(model);
            }
        }
    }
}
