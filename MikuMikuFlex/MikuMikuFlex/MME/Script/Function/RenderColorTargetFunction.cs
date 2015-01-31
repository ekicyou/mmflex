using System;
using MMF.Model;
using SlimDX.Direct3D11;

namespace MMF.MME.Script.Function
{
    internal class RenderColorTargetFunction:FunctionBase
    {
        private int index;
        private RenderContext context;
        private RenderTargetView targetView;
        private bool isDefaultTarget;

        public override string FunctionName
        {
            get { return "RenderColorTarget"; }
        }

        public override FunctionBase GetExecuterInstance(int index, string value, RenderContext context, ScriptRuntime runtime, MMEEffectManager manager, MMEEffectTechnique technique, MMEEffectPass pass)
        {
            RenderColorTargetFunction func=new RenderColorTargetFunction();
            if(index<0||index>7)throw new InvalidMMEEffectShaderException("RenderColorTarget[n](0<=n<=7)のnの制約が満たされていません。");
            func.index = index;
            func.context = context;
            if (!string.IsNullOrWhiteSpace(value) && !manager.RenderColorTargetViewes.ContainsKey(value)) throw new InvalidMMEEffectShaderException("指定されたRENDERCOLORTARGETの変数は存在しません。");
            if (string.IsNullOrWhiteSpace(value)) func.isDefaultTarget = true;
            func.targetView = string.IsNullOrWhiteSpace(value)?index==0?context.CurrentTargetContext.RenderTargetView:null:manager.RenderColorTargetViewes[value];
            return func;
        }

        public override void Execute(ISubset ipmxSubset, Action<ISubset> drawAction)
        {
            context.CurrentRenderColorTargets[index] = isDefaultTarget ? context.CurrentTargetContext.RenderTargetView : targetView;
            context.DeviceManager.Context.OutputMerger.SetTargets(context.CurrentRenderDepthStencilTarget,context.CurrentRenderColorTargets);
        }
    }
}
