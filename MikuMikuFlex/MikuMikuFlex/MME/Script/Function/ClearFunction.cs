using System;
using MMF.Model;
using SlimDX.Direct3D11;

namespace MMF.MME.Script.Function
{
    internal class ClearFunction:FunctionBase
    {
        public override string FunctionName
        {
            get { return "Clear"; }
        }

        private bool isClearDepth;

        private RenderContext context;

        private int index;

        public override FunctionBase GetExecuterInstance(int index, string value, RenderContext context, ScriptRuntime runtime, MMEEffectManager manager, MMEEffectTechnique technique, MMEEffectPass pass)
        {
            ClearFunction func=new ClearFunction();
            switch (value)
            {
                case "Color":
                    func.isClearDepth = false;
                    break;
                case "Depth":
                    func.isClearDepth = true;
                    break;
                default:
                    throw new InvalidMMEEffectShaderException(string.Format("Clear={0}が指定されましたが、\"{0}\"は指定可能ではありません。ClearもしくはDepthが指定可能です。",value));
            }
            func.context = context;
            func.index = index;
            return func;
        }

        public override void Execute(ISubset ipmxSubset, Action<ISubset> drawAction)
        {
            if (isClearDepth)
            {
                context.DeviceManager.Context.ClearDepthStencilView(context.CurrentRenderDepthStencilTarget,DepthStencilClearFlags.Depth|DepthStencilClearFlags.Stencil,context.CurrentClearDepth,0 );
            }
            else
            {
                context.DeviceManager.Context.ClearRenderTargetView(context.CurrentRenderColorTargets[index],context.CurrentClearColor);
            }
        }
    }
}
