using MMF.MME.VariableSubscriber.MaterialSubscriber;
using MMF.Model;

namespace MMF.MME.VariableSubscriber
{
    /// <summary>
    ///     エフェクト登録の際の引数
    /// </summary>
    public class SubscribeArgument
    {
        public SubscribeArgument(IDrawable model, RenderContext context)
        {
            Model = model;
            Context = context;
        }

        public SubscribeArgument(MaterialInfo info, IDrawable model, RenderContext context)
        {
            Material = info;
            Context = context;
            Model = model;
        }

        public IDrawable Model { get; private set; }

        public RenderContext Context { get; private set; }

        public MaterialInfo Material { get; private set; }
    }
}