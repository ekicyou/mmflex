using SlimDX;

namespace MMF.Bone
{
    public interface IBone
    {
        /// <summary>
        ///     平行移動位置ベクタ
        /// </summary>
        Vector3 Translation { get; set; }

        /// <summary>
        ///     回転行列
        /// </summary>
        Quaternion Rotation { get; set; }

        /// <summary>
        ///     グローバルポーズ
        /// </summary>
        Matrix GlobalPose { get; }

        void UpdateGrobalPose();
    }
}