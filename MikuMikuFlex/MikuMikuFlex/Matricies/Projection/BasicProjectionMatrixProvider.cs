using System;
using SlimDX;

namespace MMF.Matricies.Projection
{
    /// <summary>
    ///     一般的なプロジェクション行列の管理クラス
    /// </summary>
    public class BasicProjectionMatrixProvider : IProjectionMatrixProvider
    {
        private float aspectRatio = 1.618f;
        private float fovy;
        private Matrix projectionMatrix = Matrix.Identity;
        private float zFar;
        private float zNear;

        /// <summary>
        ///     プロジェクション行列
        /// </summary>
        public Matrix ProjectionMatrix
        {
            get { return projectionMatrix; }
        }

        /// <summary>
        ///     視野角(ラジアン単位)
        /// </summary>
        public float Fovy
        {
            get { return fovy; }
            set
            {
                fovy = value;
                UpdateProjection();
                NotifyProjectMatrixChanged(ProjectionMatrixChangedVariableType.Fovy);
            }
        }

        /// <summary>
        ///     アスペクト比
        /// </summary>
        public float AspectRatio
        {
            get { return aspectRatio; }
            set
            {
                aspectRatio = value;
                UpdateProjection();
                NotifyProjectMatrixChanged(ProjectionMatrixChangedVariableType.AspectRatio);
            }
        }

        /// <summary>
        ///     ニアクリップ
        /// </summary>
        public float ZNear
        {
            get { return zNear; }
            set
            {
                zNear = value;
                UpdateProjection();
                NotifyProjectMatrixChanged(ProjectionMatrixChangedVariableType.ZNear);
            }
        }

        /// <summary>
        ///     ファークリップ
        /// </summary>
        public float ZFar
        {
            get { return zFar; }
            set
            {
                zFar = value;
                UpdateProjection();
                NotifyProjectMatrixChanged(ProjectionMatrixChangedVariableType.ZFar);
            }
        }

        /// <summary>
        ///     プロジェクション行列を初期化する
        /// </summary>
        /// <param name="fovyAngle">初期視野角</param>
        /// <param name="aspect">初期アスペクト比</param>
        /// <param name="znear">初期ニアクリップ</param>
        /// <param name="zfar">初期ファークリップ</param>
        public void InitializeProjection(float fovyAngle, float aspect, float znear, float zfar)
        {
            fovy = fovyAngle;
            aspectRatio = aspect;
            zNear = znear;
            zFar = zfar;
            UpdateProjection();
        }

        public event EventHandler<ProjectionMatrixChangedEventArgs> ProjectionMatrixChanged;

        /// <summary>
        ///     プロジェクション行列を更新する
        /// </summary>
        private void UpdateProjection()
        {
            projectionMatrix = Matrix.PerspectiveFovLH(fovy, aspectRatio, zNear, zFar);
        }

        /// <summary>
        ///     プロジェクション行列が変更されたことを通知する
        /// </summary>
        /// <param name="type">変更されたパラメータ</param>
        private void NotifyProjectMatrixChanged(ProjectionMatrixChangedVariableType type)
        {
            if (ProjectionMatrixChanged != null)
                ProjectionMatrixChanged(this, new ProjectionMatrixChangedEventArgs(type));
        }
    }
}