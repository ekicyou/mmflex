using System;
using MMF.Model;
using SlimDX;

namespace MMF.Matricies.World
{
    public interface IBasicWorldMatrixProvider
    {
        Matrix getWorldMatrix(Matrix localMatrix);
    }

    /// <summary>
    ///     一般的なワールド行列の管理クラス
    /// </summary>
    public class BasicWorldMatrixProvider : IWorldMatrixProvider
    {
        /// <summary>
        ///     回転値
        /// </summary>
        private Quaternion rotation;

        /// <summary>
        ///     拡大値
        /// </summary>
        private Vector3 scaling;

        /// <summary>
        ///     平行移動量
        /// </summary>
        private Vector3 translation;

        public BasicWorldMatrixProvider()
        {
            scaling = new Vector3(1.0f, 1.0f, 1.0f);
            rotation = Quaternion.Identity;
            translation = Vector3.Zero;
        }

        public Vector3 Scaling
        {
            get { return scaling; }
            set
            {
                scaling = value;
                NotifyWorldMatrixChanged(new WorldMatrixChangedEventArgs(ChangedWorldMatrixValueType.Scaling));
            }
        }

        public Quaternion Rotation
        {
            get { return rotation; }
            set
            {
                rotation = value;
                NotifyWorldMatrixChanged(new WorldMatrixChangedEventArgs(ChangedWorldMatrixValueType.Rotation));
            }
        }

        public Vector3 Translation
        {
            get { return translation; }
            set
            {
                translation = value;
                NotifyWorldMatrixChanged(new WorldMatrixChangedEventArgs(ChangedWorldMatrixValueType.Translation));
            }
        }

        public Matrix getWorldMatrix(Vector3 scalingLocal, Quaternion rotationLocal, Vector3 translationLocal)
        {
            Vector3 scaling = new Vector3(Scaling.X*scalingLocal.X, Scaling.Y*scalingLocal.Y, Scaling.Z*scalingLocal.Z);
            return Matrix.Scaling(scaling)*Matrix.RotationQuaternion(rotationLocal*Rotation)*
                   Matrix.Translation(translationLocal + Translation);
        }

        public Matrix getWorldMatrix(Matrix localMatrix)
        {
            return Matrix.Scaling(scaling)*Matrix.RotationQuaternion(rotation)*Matrix.Translation(translation)*localMatrix;
        }

        public event EventHandler<WorldMatrixChangedEventArgs> WorldMatrixChanged;


        public Matrix getWorldMatrix(IDrawable drawable)
        {
            return getWorldMatrix(drawable.Transformer.LocalTransform);
        }

        private void NotifyWorldMatrixChanged(WorldMatrixChangedEventArgs arg)
        {
            if (WorldMatrixChanged != null) WorldMatrixChanged(this, arg);
        }
    }
}