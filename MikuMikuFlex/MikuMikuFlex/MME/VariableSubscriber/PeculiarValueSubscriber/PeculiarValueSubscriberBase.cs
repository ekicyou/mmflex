using SlimDX.Direct3D11;

namespace MMF.MME.VariableSubscriber.PeculiarValueSubscriber
{
    /// <summary>
    ///     特殊パラメータの登録クラスの基底クラス
    /// </summary>
    public abstract class PeculiarValueSubscriberBase
    {
        /// <summary>
        ///     認識される変数名
        /// </summary>
        public abstract string Name { get; }

        /// <summary>
        ///     変数型
        /// </summary>
        public abstract VariableType Type { get; }

        /// <summary>
        ///     変数に値を登録する
        /// </summary>
        /// <param name="subscribeTo">登録先の変数</param>
        /// <param name="argument">登録に利用する引数</param>
        public abstract void Subscribe(EffectVariable subscribeTo, SubscribeArgument argument);
    }
}