using MMF.Controls.Forms;

namespace _01_Initialize3DCGToYourForm
{

    //②フォームの基底クラスを変更する。
    //C#でWindowsフォームアプリケーションを作成すると以下のようなプログラムが生成される。
    //public partial class Form1:Form
    //これの部分を以下の1行のように書き換える
    public partial class Form1 : RenderForm
    {
        /*
         * ◎上の部分の解説
         * 3DCG表示する際は、RenderFormを継承したクラスを継承すると良い。
         * Formの中のオーバーライドすべき部分をオーバーライドし、適切な処理にRenderFormは書き換える。
         * (注)RenderFormの初期化はOnLoadのオーバーライドによって行われるため、OnLoadをオーバーライドする際にbase.OnLoad(e);はOnLoadの必ず最初に呼ばれるべきである。
         */
        public Form1()
        {
            InitializeComponent();
            /*
             * ｵﾏｹ
             * 背景色を変えてみる
             * 背景色を変える場合は以下のようにBackgroundColorに対して背景色をセットする。
             * new Vector3(Red値、Green値、Blue値);として渡せばよい。
             */
            //BackgroundColor = new SlimDX.Vector3(1f, 0f, 0f);
        }

    }
}
