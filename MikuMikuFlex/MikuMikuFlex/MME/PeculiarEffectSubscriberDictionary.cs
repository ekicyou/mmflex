using System.Collections.Generic;
using MMF.MME.VariableSubscriber.PeculiarValueSubscriber;

namespace MMF.MME
{
    public class PeculiarEffectSubscriberDictionary : Dictionary<string, PeculiarValueSubscriberBase>
    {
        public void Add(PeculiarValueSubscriberBase subscriber)
        {
            base.Add(subscriber.Name, subscriber);
        }
    }
}