using System;
using SlimDX;
using SlimDX.Direct3D11;

namespace MMF.Matricies.Camera
{
    /// <summary>
    ///     標準的なカメラ管理クラス
    /// </summary>
    public class BasicCamera : CameraProvider
    {
        private Vector3 cameraLookAt = Vector3.Zero;
        private Vector3 cameraPosition = Vector3.Zero;
        private Vector3 cameraUpVec = Vector3.Zero;
        private Matrix viewMatrix = Matrix.Identity;

        /// <summary>
        ///     コンストラクタ
        /// </summary>
        /// <param name="cameraPos">カメラ初期座標</param>
        /// <param name="lookAtPos">カメラ初期視点座標</param>
        /// <param name="upVec">カメラ初期上方向ベクトル</param>
        public BasicCamera(Vector3 cameraPos, Vector3 lookAtPos, Vector3 upVec)
            : base(cameraPos, lookAtPos, upVec)
        {
        }

        /// <summary>
        ///     このクラスのメンバから計算されるビュー行列
        /// </summary>
        public override Matrix ViewMatrix
        {
            get { return viewMatrix; }
        }


        /// <summary>
        ///     カメラ位置
        /// </summary>
        public override Vector3 CameraPosition
        {
            get { return cameraPosition; }
            set
            {
                cameraPosition = value;
                UpdateCamera();
                NotifyCameraMatrixChanged(CameraMatrixChangedVariableType.Position);
            }
        }


        /// <summary>
        ///     カメラの注視点
        /// </summary>
        public override Vector3 CameraLookAt
        {
            get { return cameraLookAt; }
            set
            {
                cameraLookAt = value;
                UpdateCamera();
                NotifyCameraMatrixChanged(CameraMatrixChangedVariableType.LookAt);
            }
        }


        /// <summary>
        ///     カメラの上方向
        /// </summary>
        public override Vector3 CameraUpVec
        {
            get { return cameraUpVec; }
            set
            {
                cameraUpVec = value;
                UpdateCamera();
                NotifyCameraMatrixChanged(CameraMatrixChangedVariableType.Up);
            }
        }

        /// <summary>
        ///     ビュー行列を更新する
        /// </summary>
        private void UpdateCamera()
        {
            viewMatrix = Matrix.LookAtLH(cameraPosition, cameraLookAt, cameraUpVec);
        }

        /// <summary>
        ///     ビュー行列をエフェクトに登録
        /// </summary>
        /// <param name="effect"></param>
        public override void SubscribeToEffect(Effect effect)
        {
            effect.GetVariableBySemantic("CAMERAPOSITION").AsVector().Set(cameraPosition);
        }

        /// <summary>
        ///     カメラの行列が変更されたことを通知します
        /// </summary>
        /// <param name="type">変更内容</param>
        private void NotifyCameraMatrixChanged(CameraMatrixChangedVariableType type)
        {
            if (CameraMatrixChanged != null) CameraMatrixChanged(this, new CameraMatrixChangedEventArgs(type));
        }

        /// <summary>
        ///     ビュー行列が変更された際に通知されます
        /// </summary>
        public override event EventHandler<CameraMatrixChangedEventArgs> CameraMatrixChanged;
    }
}