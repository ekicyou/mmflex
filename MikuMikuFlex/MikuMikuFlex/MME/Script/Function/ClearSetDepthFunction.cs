using System;
using MMF.Model;
using SlimDX.Direct3D11;

namespace MMF.MME.Script.Function
{
    internal class ClearSetDepthFunction:FunctionBase
    {
        public override string FunctionName
        {
            get { return "ClearSetDepth"; }
        }

        private RenderContext context;

        private EffectVariable sourceVariable;

        public override FunctionBase GetExecuterInstance(int index, string value, RenderContext context, ScriptRuntime runtime, MMEEffectManager manager, MMEEffectTechnique technique, MMEEffectPass pass)
        {
            ClearSetDepthFunction func=new ClearSetDepthFunction();
            func.context = context;
            func.sourceVariable = manager.EffectFile.GetVariableByName(value);
            if (func.sourceVariable == null) throw new InvalidMMEEffectShaderException(string.Format("ClearSetDepth={0};が指定されましたが、変数\"{0}\"は見つかりませんでした。", value));
            if (!func.sourceVariable.GetVariableType().Description.TypeName.ToLower().Equals("float"))
                throw new InvalidMMEEffectShaderException(string.Format("ClearSetDepth={0};が指定されましたが、変数\"{0}\"はfloat型ではありません。", value));
            return func;
        }

        public override void Execute(ISubset ipmxSubset, Action<ISubset> drawAction)
        {
            context.CurrentClearDepth = sourceVariable.AsScalar().GetFloat();
        }
    }
}
