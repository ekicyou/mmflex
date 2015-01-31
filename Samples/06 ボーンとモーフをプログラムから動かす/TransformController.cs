using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MMF.Bone;
using MMF.Model.PMX;
using SlimDX;

namespace _06_MoveBoneAndMorphFromCode
{
    public partial class TransformController : Form
    {

        //②-B参照
        private ManualTransformUpdater updater;

        //ボーン回転の1回当たりの回転量
        private readonly float rotationAngle = (float) (Math.PI/24f);

        public TransformController()
        {
            InitializeComponent();
        }

        public TransformController(PMXModel model):this()
        {
            //②-Aコンボボックスにボーン名、モーフ名を登録する
            foreach (var boneData in model.Model.BoneList.Bones)
            {
                bone_combo_box.Items.Add(boneData.BoneName);
            }
            foreach (var morphData in model.Model.MorphList.Morphes)
            {
                morph_combo_box.Items.Add(morphData.MorphName);
            }
            /*
             * MMDModel.Modelにはモデルをパースしたデータそのままが入っている。
             * データ上はMMDのデータをすべてロードしているはずなので、もしMMFで対応していない機能などがあった場合は、MMDModel.Model
             * からデータを取得することが可能。
             */

            //②-B マニュアルで変更するためのボーンやモーフを更新するクラスを作成する。
            updater = new ManualTransformUpdater(model);

            //②-C ②-Bで作成したクラスを変形順序内に入れる。
            model.Skinning.KinematicsProviders[0] = updater;
            /*
             * MMDのモデルはいくつかのフェイズごとに変換される。
             * たとえば、モーション変形→IK変形→物理変形という具合である。
             * MMDModel.Skinning.KinematicsProvidersにはこの順序が記録されている。
             * 通常は、MMDModelWithPhysicsで作成したモデルは、
             * 0 モーションの変形クラス
             * 1 IK変形クラス
             * 2 物理演算クラス
             * という順番になっている。
             * 今回は、モーションを使わないので、0番を入れ替えて使う。もちろん、Listなので挿入することもできる
             */
        }
        #region ②-D ボタンによるボーン回転

        private void bone_x_plus_Click(object sender, EventArgs e)
        {
            if (string.IsNullOrEmpty((string)bone_combo_box.SelectedItem)) return;
            BoneTransformer transformer=updater.getBoneTransformer((string) bone_combo_box.SelectedItem);
            transformer.Rotation *= Quaternion.RotationAxis(new Vector3(1, 0, 0), rotationAngle);
        }

        private void bone_y_plus_Click(object sender, EventArgs e)
        {
            if (string.IsNullOrEmpty((string)bone_combo_box.SelectedItem)) return;
            BoneTransformer transformer = updater.getBoneTransformer((string)bone_combo_box.SelectedItem);
            transformer.Rotation *= Quaternion.RotationAxis(new Vector3(0, 1, 0), rotationAngle);
        }

        private void bone_z_plus_Click(object sender, EventArgs e)
        {
            if (string.IsNullOrEmpty((string)bone_combo_box.SelectedItem)) return;
            BoneTransformer transformer = updater.getBoneTransformer((string)bone_combo_box.SelectedItem);
            transformer.Rotation *= Quaternion.RotationAxis(new Vector3(0, 0, 1), rotationAngle);
        }

        private void bone_z_minus_Click(object sender, EventArgs e)
        {
            if (string.IsNullOrEmpty((string)bone_combo_box.SelectedItem)) return;
            BoneTransformer transformer = updater.getBoneTransformer((string)bone_combo_box.SelectedItem);
            transformer.Rotation *= Quaternion.RotationAxis(new Vector3(0, 0, 1), -rotationAngle);
        }

        private void bone_y_minus_Click(object sender, EventArgs e)
        {
            if (string.IsNullOrEmpty((string)bone_combo_box.SelectedItem)) return;
            BoneTransformer transformer = updater.getBoneTransformer((string)bone_combo_box.SelectedItem);
            transformer.Rotation *= Quaternion.RotationAxis(new Vector3(0, 1, 0), -rotationAngle);
        }

        private void bone_x_minus_Click(object sender, EventArgs e)
        {
            if (string.IsNullOrEmpty((string)bone_combo_box.SelectedItem)) return;
            BoneTransformer transformer = updater.getBoneTransformer((string)bone_combo_box.SelectedItem);
            transformer.Rotation *= Quaternion.RotationAxis(new Vector3(1, 0, 0), -rotationAngle);
        }
        /*
         * ボーンの回転をボタンに応じて行う部分。
         * 1行目：if (string.IsNullOrEmpty((string)bone_combo_box.SelectedItem)) return;
         * コンボボックスで選ばれているモーフが存在しないときは処理をしない。
         * 2行目：BoneTransformer transformer = updater.getBoneTransformer((string)bone_combo_box.SelectedItem);
         * コンストラクタで作成したManualTransformUpdaterからボーンの変形クラスを取得する。
         * 3行目:transformer.Rotation *= Quaternion.RotationAxis(回転軸方向ベクトル,回転角);
         * 2行目で取得したRotationに対して掛け合わせる。ManualTransformUpdaterによる更新では、フレームごとに変化量はリセットされない。
         * よって、変形の必要な時に追加で掛け合わせて行けばよく、毎フレームごと現在の回転量を代入する必要はない。
         * 正確には、ボーンの回転量などは毎フレームごと内部的にリセットしているが、ManualTransformUpdaterは内部で毎回、インスタンスが作られたBoneTransformerの値をセットしている。
         * なお、getBoneTransformerで同じボーン名を複数回取得しても、同じインスタンスが返される。このため、別々の場所から変形を掛け合わせると合成される。
         */
        #endregion

        //②-E モーフの値をセットする
        private void morph_track_bar_ValueChanged(object sender, EventArgs e)
        {
            if(string.IsNullOrEmpty((string)morph_combo_box.SelectedItem))return;
            MorphTransformer transformer = updater.getMorphTransformer((string) morph_combo_box.SelectedItem);
            transformer.MorphValue = (float) morph_track_bar.Value/morph_track_bar.Maximum;
        }
        /*
        * 1.2行目は基本的に②-Dと同じであるが、モーフの場合は、getMorphTransformerで、MorphTransformerを取得する。
        * この値のメンバのMorphValueにセットすることで値を変えることが可能。
        * PMXの仕様ではモーフの値が0以上1以下以外はサポートされないが、MMFでは代入することは可能である。ただしその動作については保証しない。
        * ManualTransformUpdaterによる更新では、フレームごとに変化量はリセットされない。
        * よって、変形の必要な時に追加で掛け合わせて行けばよく、毎フレームごと現在のモーフ値を代入する必要はない。
        */

        //②-F コンボボックスの値が変わったときにトラックバーの値を対応したモーフの値にする。
        private void morph_combo_box_SelectedValueChanged(object sender, EventArgs e)
        {
            if(string.IsNullOrEmpty((string)morph_combo_box.SelectedItem))return;
            MorphTransformer transformer = updater.getMorphTransformer((string)morph_combo_box.SelectedItem);
            morph_track_bar.Value = (int) (transformer.MorphValue*morph_track_bar.Maximum);
        }
        /*
         * ②-Eと同様の手順でモーフの変形量を取得することが可能
         */


    }
}
