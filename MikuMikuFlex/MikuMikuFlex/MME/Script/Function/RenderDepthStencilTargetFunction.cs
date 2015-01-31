using System;
using MMF.Model;
using SlimDX.Direct3D11;

namespace MMF.MME.Script.Function
{
    internal class RenderDepthStencilTargetFunction:FunctionBase
    {
        private DepthStencilView stencilView;

        private RenderContext context;

        private bool isDefaultTarget;

        public override string FunctionName
        {
            get { return "RenderDepthStencilTarget"; }
        }

        public override FunctionBase GetExecuterInstance(int index, string value, RenderContext context, ScriptRuntime runtime, MMEEffectManager manager, MMEEffectTechnique technique, MMEEffectPass pass)
        {
            RenderDepthStencilTargetFunction func=new RenderDepthStencilTargetFunction();
            if(index!=0)throw new InvalidMMEEffectShaderException("RenderDepthStencilTargetにはインデックス値を指定できません。");
            func.context = context;
            if(!string.IsNullOrWhiteSpace(value)&&!manager.RenderDepthStencilTargets.ContainsKey(value))throw new InvalidMMEEffectShaderException("スクリプトに指定された名前の深度ステンシルバッファは存在しません。");
            /*
             * valueが空なら→デフォルトの深度ステンシルバッファ
             * valueがあるなら→その名前のテクスチャ変数から取得
             */
            if (string.IsNullOrWhiteSpace(value)) func.isDefaultTarget = true;
            func.stencilView =string.IsNullOrWhiteSpace(value)?null: manager.RenderDepthStencilTargets[value];
            return func;
        }

        public override void Execute(ISubset ipmxSubset, Action<ISubset> drawAction)
        {
            context.CurrentRenderDepthStencilTarget=isDefaultTarget?context.CurrentTargetContext.DepthTargetView:stencilView;
            context.DeviceManager.Context.OutputMerger.SetTargets(context.CurrentRenderDepthStencilTarget,context.CurrentRenderColorTargets);
        }
    }
}
