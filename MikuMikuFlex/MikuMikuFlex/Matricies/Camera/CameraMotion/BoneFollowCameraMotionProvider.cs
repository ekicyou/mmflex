using System;
using System.Linq;
using MMF.Bone;
using MMF.Matricies.Projection;
using MMF.Model.PMX;
using SlimDX;

namespace MMF.Matricies.Camera.CameraMotion
{
    /// <summary>
    /// ボーン追従カメラモーション
    /// camera motion tracking model bone
    /// </summary>
    public class BoneFollowCameraMotionProvider:ICameraMotionProvider
    {
        /// <summary>
        /// トラッキング対象のモデル
        /// </summary>
        private readonly PMXModel followModel;

        /// <summary>
        /// トラッキング対象のボーン
        /// </summary>
        private readonly PMXBone followBone;

        /// <summary>
        /// カメラとボーンとの距離
        /// Distance of camera and bone
        /// </summary>
        public float Distance { get; set; }

        /// <summary>
        /// カメラから見てZ軸方向に回転するかどうか
        /// </summary>
        public bool IsRotationZAxis { get; set; }

        /// <summary>
        /// 初期状態の時から見てどっちから見るか。
        /// (0,0,1)ならば前から、(0,0,-1)ならば後ろから
        /// 
        /// </summary>
        public Vector3 ViewFrom { get; set; }

        /// <summary>
        /// Constractor
        /// 
        /// </summary>
        /// <param name="model"></param>
        /// <param name="boneName"></param>
        /// <param name="distance"></param>
        /// <param name="viewFrom"></param>
        /// <param name="rotationZaxis"></param>
        public BoneFollowCameraMotionProvider(PMXModel model,string boneName,float distance,Vector3 viewFrom,bool rotationZaxis=false)
        {
            followModel = model;
            var bones= (from bone in model.Skinning.Bone where bone.BoneName == boneName select bone).ToArray();
            if (bones.Length == 0)
            {
                throw new InvalidOperationException(string.Format("ボーン\"{0}\"は見つかりませんでした。",boneName));
            }
            followBone = bones[0];
            Distance = distance;
            IsRotationZAxis = rotationZaxis;
            ViewFrom = viewFrom;
        }


        void ICameraMotionProvider.UpdateCamera(CameraProvider cp, IProjectionMatrixProvider proj)
        {
            //ボーンのワールド座標を求める行列を作成
            Matrix bonePoseMatrix = followBone.GlobalPose*Matrix.Scaling(followModel.Transformer.Scale)*
                                    Matrix.RotationQuaternion(followModel.Transformer.Rotation)*
                                    Matrix.Translation(followModel.Transformer.Position);
            Vector3 bonePosition = Vector3.TransformCoordinate(followBone.Position, bonePoseMatrix);
            Vector3 la2cp = Vector3.TransformNormal(-ViewFrom, bonePoseMatrix);//注視点から、カメラの場所に向かうベクトル
            la2cp.Normalize();
            cp.CameraPosition = bonePosition + Distance*la2cp;
            cp.CameraLookAt = bonePosition;
            if (IsRotationZAxis)
            {
                Vector3 newUp = Vector3.TransformNormal(new Vector3(0, 1, 0), bonePoseMatrix);
                newUp.Normalize();
                cp.CameraUpVec = newUp;
            }
        }
    }
}
