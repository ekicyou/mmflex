using System;
using System.Windows.Forms;
using MMF;
using MMF.Controls.Forms;
using MMF.Model.PMX;

namespace _02_SimpleRenderPMX
{
    public partial class Form1 : RenderForm
    {
        public Form1()
        {
            InitializeComponent();
        }

        //①Form.OnLoadをオーバーライドする。
        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e); //RenderFormはOnLoad内で3DCG空間を初期化しているため、base.OnLoadがOnLoad内で一番初めに呼ぶべきである。

            //ファイルを開くダイアログ
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Filter = "pmxモデルファイル(*.pmx)|*.pmx";
            if (ofd.ShowDialog() == DialogResult.OK)
            {
            //ダイアログの返値がOKの場合、モデルの読み込み処理をする


                //②モデルを読み込む
                PMXModel model = PMXModelWithPhysics.OpenLoad(ofd.FileName, RenderContext);//MMDModel MMDModelWithPhysics.OpenLoad(string ファイル名,RenderContext);となっています
                //以下のように書けば、物理演算を無効にして読み込むことも可能
                //MMDModel model=MMDModel.OpenLoad(ofd.FileName, RenderContext);
                //RenderContextはカメラの情報やデバイスの情報など3DCG描画に必要な変数である。RenderFormを継承している場合、メンバー変数として利用可能である。
                //基本的にモデルなどの読み込みが必要なものは、DirectX11デバイスの情報などを保持するRenderContextの値を要求する場合が多い。

                //③ワールド空間にモデルを追加する
                WorldSpace.AddResource(model);
                //WorldSpaceは、このフォームの描画する3D空間を示している。ここにモデルなど(IDrawableを実装している)ものを渡すと、描画してくれる。
                //WorldSpaceは、ScreenContext.WorldSpaceと常に等しい。ウィンドウごとに必要な3DCG描画に必要な情報はScreenContextに保管されている。
            }
        }
    }
}