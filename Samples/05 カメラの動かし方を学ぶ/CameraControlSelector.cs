using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MMF.Matricies.Camera;
using MMF.Matricies.Camera.CameraMotion;
using MMF.Model.PMX;
using SlimDX;

namespace _05_HowToUpdateCamera
{
    public partial class CameraControlSelector : Form
    {
        private PMXModel Model;

        public CameraControlSelector()
        {
            InitializeComponent();
        }

        public CameraControlSelector(PMXModel model) : this()
        {
            this.Model = model;
        }

        //①-A このダイアログの結果を代入するメンバの配置
        public ICameraMotionProvider ResultCameraMotionProvider;

        #region ①-Bボタンに応じて適切なカメラモーションの操作クラスを割り当てる
        private void basic_camera_controller_Click(object sender, EventArgs e)
        {
            ResultCameraMotionProvider = new BasicCameraControllerMotionProvider(Owner, Owner);
            Close();
            /*
             * BasicCameraControllerMotionProviderはMMFで定義済みのカメラコントローラーである。
             * いまだに操作性がMMDやMMMのカメラ操作と比べると低いが、マウスによりカメラを動かす処理を行うクラスである。
             * コンストラクタの引数は、第一引数がマウスのイベントを受け取る相手、第二引数がマウスのホイールのイベントを受け取る相手、第三引数が初期状態でのカメラと注視点の距離である。
             * 第三引数は初期状態で5.0fとなっている。
             * 第一引数、第二引数は多くの場合同じものをさすが、レンダリング対象がユーザーコントロールの場合、第一引数はユーザーコントロール、第二引数はそれを含むフォームを渡さなければ、正常にイベントを取得できずまともに動作しない。
             */
        }

        private void vmd_camera_controller_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofd=new OpenFileDialog();
            ofd.Filter = "vmdカメラモーションファイル(*vmd)|*vmd";
            if (ofd.ShowDialog(this) == DialogResult.OK)
            {
                VMDCameraMotionProvider prov = VMDCameraMotionProvider.OpenFile(ofd.FileName);
                prov.Start();
                ResultCameraMotionProvider = prov;
                /*
                 * VMDCameraMotionProviderはMMFで定義済みのカメラコントローラーである。
                 * VMDファイルによってカメラを動かす場合に利用する。
                 * コンストラクタを利用し、ストリームなどから初期化することは可能であるがここでは便利な静的メソッドを利用して初期化するものとする。
                 * OpenFileはstring形式のパスを受け取る。Startメソッドを呼べばカメラモーションは再生される。
                 */
            }
            Close();
        }

        private void bone_tracker_Click(object sender, EventArgs e)
        {
            ResultCameraMotionProvider=new BoneFollowCameraMotionProvider(Model,"頭",10f,new Vector3(0,0,1),false);
            Close();
            /*
             * BoneFollowCameraMotionProviderはMMFで定義済みのカメラコントローラーである。
             * ボーンを追従するカメラを利用する際に利用する。
             * 引数は以下のとおりである。
             * 1,ターゲットのモデル
             * 2,ターゲットのボーンの名前
             * 3,ボーンから維持する距離
             * 4,モデルから見てどちらにカメラがあるかを指す。Z=1なら、奥行きてきにモデルの正面、Z=-1なら奥行き的にモデルの後ろとなる。
             * 5,Z軸方向の回転をするかどうか。首をかしげる動きなどで首にバインドしたとき横のねじりまでカメラ側が追従するかどうかを指す。
             */
        }

        private void user_definition_Click(object sender, EventArgs e)
        {
            ResultCameraMotionProvider=new SimpleUserDefinitionCameraMotionProvider();
            Close();
            /*
             * カメラモーションについてはカンタンに定義することが可能。
             * 定義の仕方についてはSimpleUserDefinitionCameraMotionProviderを参照すると良い。
             */
        }
        #endregion
    }
}
