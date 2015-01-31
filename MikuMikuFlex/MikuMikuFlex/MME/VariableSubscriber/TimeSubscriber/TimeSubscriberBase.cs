using MMF.Model.PMX;
using MMF.Motion;
using SlimDX.Direct3D11;

namespace MMF.MME.VariableSubscriber.TimeSubscriber
{
    public abstract class TimeSubscriberBase : SubscriberBase
    {
        /// <summary>
        ///     MME仕様2.4参照
        /// </summary>
        protected bool SyncInEditMode;

        protected TimeSubscriberBase(bool syncInEditMode)
        {
            SyncInEditMode = syncInEditMode;
        }

        protected TimeSubscriberBase()
        {
        }

        public override VariableType[] Types
        {
            get { return new[] {VariableType.Float}; }
        }


        public override SubscriberBase GetSubscriberInstance(EffectVariable variable, RenderContext context, MMEEffectManager effectManager, int semanticIndex)
        {
            EffectVariable annotation = variable.GetAnnotationByName("SyncInEditMode");
            bool syncMode = false;
            if (annotation != null) syncMode = annotation.AsScalar().GetInt() == 1;
            return GetSubscriberInstance(syncMode);
        }

        protected abstract SubscriberBase GetSubscriberInstance(bool syncInEditMode);

        private PMXModel modelCache;

        private bool isCached;

        public override void Subscribe(EffectVariable subscribeTo, SubscribeArgument variable)
        {
            if (!isCached)
            {
                modelCache = variable.Model as PMXModel;
                isCached = true;
            }

            if(modelCache!=null)Subscribe(subscribeTo,modelCache.MotionManager, variable.Context);
        }

        protected abstract void Subscribe(EffectVariable variable, IMotionManager motion, RenderContext context);
    }
}