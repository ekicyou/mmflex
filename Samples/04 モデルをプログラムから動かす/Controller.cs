using System;
using System.Windows.Forms;
using MMF.Model.PMX;
using SlimDX;

namespace _04_TransformModelFormCode
{
    //①コントロールするためのGUIを作成する。
    public partial class Controller : Form
    {
        public Controller()
        {
            InitializeComponent();
        }

        //①-A 操作対象のモデルを格納しておく変数や、一回のボタンによる変異の量を格納しておく定数を作成する
        public PMXModel Model { get; set; }

        private readonly float translationLength = 1f;

        private readonly float rotationAngle = (float) (Math.PI/24f);

        public Controller(PMXModel model):this()
        {
            //①-B コンストラクタで受け取ったら変数に代入
            this.Model = model;
        }


        #region ①-C ボタンに応じて平行移動する
        private void trans_x_plus_Click(object sender, System.EventArgs e)
        {
            Model.Transformer.Position+=new Vector3(translationLength,0,0);
        }

        private void trans_y_plus_Click(object sender, EventArgs e)
        {
            Model.Transformer.Position += new Vector3(0, translationLength, 0);
        }

        private void trans_z_plus_Click(object sender, EventArgs e)
        {
            Model.Transformer.Position += new Vector3(0,0, translationLength);
        }

        private void trans_x_minus_Click(object sender, EventArgs e)
        {
            Model.Transformer.Position += new Vector3(-translationLength, 0, 0);
        }

        private void trans_y_minus_Click(object sender, EventArgs e)
        {
            Model.Transformer.Position += new Vector3(0, -translationLength, 0);
        }

        private void trans_z_minus_Click(object sender, EventArgs e)
        {
            Model.Transformer.Position += new Vector3(0, 0, -translationLength);
        }

        /*
         * 説明：
         * モデルに関する平行移動、回転、拡大はすべてTransformerを介して行われる。
         * TransformerはMMDModelだけでなく描画可能なIDrawableインターフェースを実装するすべてのクラスに含まれるため、
         * 今後のチュートリアルに登場するグリッドやXファイルモデルなどについても同様に平行移動、回転、拡大が可能である。
         * 
         * 平行移動はPositionを利用する。これはワールド座標系による座標系です。
         * 平行移動の値はベクトルのためベクトルの知識があると、この部分の理解は容易です。
         * また、回転はクォータニオン、拡大はベクトルで管理しています。
         * 拡大についてはTransformer.Scaleに対してX軸方向、Y軸方向、Z軸方向への拡大率を代入することで可能です。
         * ただし、エフェクト側での拡大率取得がスカラー値のため、X軸方向、Y軸方向、Z軸方向に別の拡大率を与えた場合はエフェクトの正常動作は保障されません。
         */
        #endregion

        #region ①-D ボタンに応じて回転させる

        private void rotate_x_plus_Click(object sender, EventArgs e)
        {
            Model.Transformer.Rotation *= Quaternion.RotationAxis(new Vector3(1, 0, 0), rotationAngle);
        }

        private void rotate_y_plus_Click(object sender, EventArgs e)
        {
            Model.Transformer.Rotation *= Quaternion.RotationAxis(new Vector3(0, 1, 0), rotationAngle);
        }

        private void rotate_z_plus_Click(object sender, EventArgs e)
        {
            Model.Transformer.Rotation *= Quaternion.RotationAxis(new Vector3(0, 0, 1), rotationAngle);
        }

        private void rotate_x_minus_Click(object sender, EventArgs e)
        {
            Model.Transformer.Rotation *= Quaternion.RotationAxis(new Vector3(1, 0, 0), -rotationAngle);
        }

        private void rotate_y_minus_Click(object sender, EventArgs e)
        {
            Model.Transformer.Rotation *= Quaternion.RotationAxis(new Vector3(0, 1, 0), -rotationAngle);
        }

        private void rotate_z_minus_Click(object sender, EventArgs e)
        {
            Model.Transformer.Rotation *= Quaternion.RotationAxis(new Vector3(0, 0, 1), -rotationAngle);
        }
        /*
         * 説明:
         * Rotationは平行移動と同様にTransformerを介して行われる。
         * 回転にはクォータニオン(４元数)と呼ばれる表現が使われていて、数学的知識が必要のように見えるが表面上だけでの理解は容易である。
         * Quaternion.RotationAxisは指定されたベクトルを軸として指定した回転量を持つクォータニオンを生成する。
         * クォータニオンはこの軸中心にこれだけ回転するよ！って値だと思ってください。ベクトルの場合、ここを中心としてこれだけ移動するよ！って値であると同じことです。
         * ベクトルは二つの値を足すことで合成しますが、クォータニオンは掛け算で合成します。
         * 回転量を足していくような操作が掛け算になると思ってください。
         * そのため、Model.Transformer.Rotation  *=となっています。
         */
        #endregion

    }
}
