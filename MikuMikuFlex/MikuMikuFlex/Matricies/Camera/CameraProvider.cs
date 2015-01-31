using System;
using SlimDX;
using SlimDX.Direct3D11;

namespace MMF.Matricies.Camera
{
    /// <summary>
    ///     カメラ管理クラスの基底クラス
    /// </summary>
    public abstract class CameraProvider
    {
        /// <summary>
        ///     コンストラクタ
        /// </summary>
        /// <param name="cameraPos">カメラ初期位置</param>
        /// <param name="lookAtPos">カメラ初期視点位置</param>
        /// <param name="upVec">カメラ初期上方向ベクトル</param>
        public CameraProvider(Vector3 cameraPos, Vector3 lookAtPos, Vector3 upVec)
        {
            CameraPosition = cameraPos;
            CameraLookAt = lookAtPos;
            CameraUpVec = upVec;
        }

        /// <summary>
        ///     カメラ座標
        /// </summary>
        public abstract Vector3 CameraPosition { get; set; }

        /// <summary>
        ///     カメラ注視点座標
        /// </summary>
        public abstract Vector3 CameraLookAt { get; set; }

        /// <summary>
        ///     カメラ上方向ベクトル
        /// </summary>
        public abstract Vector3 CameraUpVec { get; set; }

        /// <summary>
        ///     ビュー行列
        /// </summary>
        public abstract Matrix ViewMatrix { get; }

        /// <summary>
        ///     エフェクトに登録するもの
        /// </summary>
        /// <param name="effect"></param>
        public virtual void SubscribeToEffect(Effect effect)
        {
            effect.GetVariableBySemantic("CAMERAPOSITION").AsVector().Set(CameraPosition);
        }

        /// <summary>
        ///     ビュー行列が更新されたことを通知する
        /// </summary>
        public abstract event EventHandler<CameraMatrixChangedEventArgs> CameraMatrixChanged;
    }
}