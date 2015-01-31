using MMF.Model;
using MMF.Morph;

namespace MMF.Bone
{
    public interface ITransformUpdater
    {
        /// <summary>
        /// ボーンの変化量などを計算し、ボーンに割り当てる
        /// </summary>
        /// <returns>この値を元に行列を即生成するかどうか</returns>
        bool UpdateTransform();
    }
}