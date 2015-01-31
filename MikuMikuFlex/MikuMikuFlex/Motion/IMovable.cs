using MMF.Bone;

namespace MMF.Motion
{
    public interface IMovable
    {
        /// <summary>
        /// 動かす対象のボーン
        /// </summary>
        ISkinningProvider Skinning { get; }

        /// <summary>
        /// 動かす対象のモーションマネージャ
        /// </summary>
        IMotionManager MotionManager { get; }

        /// <summary>
        /// 動かす対象のモーションをフレームごとに割り当てる際に利用する。
        /// </summary>
        void ApplyMove();
    }
}
