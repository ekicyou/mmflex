using System;
using SlimDX;

namespace MMF.Matricies.Projection
{
    /// <summary>
    ///     プロジェクション行列のインターフェース
    /// </summary>
    public interface IProjectionMatrixProvider
    {
        /// <summary>
        ///     プロジェクション行列
        /// </summary>
        Matrix ProjectionMatrix { get; }

        /// <summary>
        ///     視野角
        /// </summary>
        float Fovy { get; set; }

        /// <summary>
        ///     アスペクト比
        /// </summary>
        float AspectRatio { get; set; }

        /// <summary>
        ///     ニアクリップ
        /// </summary>
        float ZNear { get; set; }

        /// <summary>
        ///     ファークリップ
        /// </summary>
        float ZFar { get; set; }

        /// <summary>
        ///     プロジェクション行列を初期化する
        /// </summary>
        /// <param name="fovyAngle">初期視野角</param>
        /// <param name="aspect">初期アスペクト比</param>
        /// <param name="znear">初期ニアクリップ</param>
        /// <param name="zfar">初期ファークリップ</param>
        void InitializeProjection(float fovyAngle, float aspect, float znear, float zfar);

        /// <summary>
        ///     プロジェクション行列が変更されたことを通知する
        /// </summary>
        event EventHandler<ProjectionMatrixChangedEventArgs> ProjectionMatrixChanged;
    }
}