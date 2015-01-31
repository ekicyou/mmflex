using System.Collections.Generic;
using MMF.MME.VariableSubscriber;

namespace MMF.MME
{
    /// <summary>
    ///     エフェクトのセマンティクスに応じて登録するクラス用のディクショナリ
    /// </summary>
    public class EffectSubscriberDictionary : Dictionary<string, SubscriberBase>
    {
        /// <summary>
        ///     追加、キーはセマンティクスとなる
        /// </summary>
        /// <param name="subs">登録するSubscriberBase</param>
        public void Add(SubscriberBase subs)
        {
            base.Add(subs.Semantics, subs);
        }
    }
}