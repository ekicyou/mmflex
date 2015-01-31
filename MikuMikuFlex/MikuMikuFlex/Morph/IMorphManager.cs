using System.Collections.Generic;
using MMF.Motion;

namespace MMF.Morph
{
    /// <summary>
    /// モーフの管理クラスのインターフェース
    /// </summary>
    public interface IMorphManager
    {
        /// <summary>
        /// モーフのモーションから再生
        /// </summary>
        /// <param name="frame"></param>
        /// <param name="morphs"></param>
        void ApplyMorphProgress(float frame, IEnumerable<MorphMotion> morphs);

        /// <summary>
        /// 指定したモーフのフレームをセットする
        /// </summary>
        /// <param name="frame"></param>
        /// <param name="morphName"></param>
        void ApplyMorphProgress(float frame, string morphName);

        /// <summary>
        /// フレームがリセットされるとき(Updateのタイミングで呼び出す)
        /// </summary>
        void UpdateFrame();


        float getMorphProgress(string morphName);
    }
}