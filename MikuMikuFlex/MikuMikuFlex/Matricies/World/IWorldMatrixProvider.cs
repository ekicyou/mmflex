using System;
using MMF.Model;
using SlimDX;

namespace MMF.Matricies.World
{
    /// <summary>
    ///     ワールド行列のインターフェース
    /// </summary>
    public interface IWorldMatrixProvider
    {
        /// <summary>
        ///     拡大率
        /// </summary>
        Vector3 Scaling { get; set; }

        /// <summary>
        ///     回転
        /// </summary>
        Quaternion Rotation { get; set; }

        /// <summary>
        ///     平行移動
        /// </summary>
        Vector3 Translation { get; set; }

        /// <summary>
        ///     ローカル値とあわせてワールド変換行列を作成します
        /// </summary>
        /// <param name="scalingLocal">ローカル拡大</param>
        /// <param name="rotationLocal">ローカル回転</param>
        /// <param name="translationLocal">ローカル平行移動</param>
        /// <returns></returns>
        Matrix getWorldMatrix(Vector3 scalingLocal, Quaternion rotationLocal, Vector3 translationLocal);

        /// <summary>
        ///     指定したモデルのワールド変換行列を作成します。
        /// </summary>
        /// <param name="drawable">取得したいローカル座標などを含むモデル</param>
        /// <returns>値</returns>
        Matrix getWorldMatrix(IDrawable drawable);

         Matrix getWorldMatrix(Matrix localMatrix);

        /// <summary>
        ///     ワールド行列が変更されたことを通知します
        /// </summary>
        event EventHandler<WorldMatrixChangedEventArgs> WorldMatrixChanged;
    }
}