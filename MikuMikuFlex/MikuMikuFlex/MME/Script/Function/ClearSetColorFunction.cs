using System;
using MMF.Model;
using SlimDX;
using SlimDX.Direct3D11;

namespace MMF.MME.Script.Function
{
    internal class ClearSetColorFunction:FunctionBase
    {
        public override string FunctionName
        {
            get { return "ClearSetColor"; }
        }

        private EffectVariable sourceVariable;//不変なら先に取得スべき？

        private RenderContext context;

        public override FunctionBase GetExecuterInstance(int index, string value, RenderContext context, ScriptRuntime runtime,
            MMEEffectManager manager,MMEEffectTechnique technique,MMEEffectPass pass)
        {
            ClearSetColorFunction func=new ClearSetColorFunction();
            func.sourceVariable=manager.EffectFile.GetVariableByName(value);
            func.context = context;
            if(func.sourceVariable==null)throw new InvalidMMEEffectShaderException(string.Format("ClearSetColor={0};が指定されましたが、変数\"{0}\"は見つかりませんでした。",value));
            if(!func.sourceVariable.GetVariableType().Description.TypeName.ToLower().Equals("float4"))
                throw new InvalidMMEEffectShaderException(string.Format("ClearSetColor={0};が指定されましたが、変数\"{0}\"はfloat4型ではありません。",value));
            return func;
        }

        public override void Execute(ISubset ipmxSubset, Action<ISubset> drawAction)
        {
            context.CurrentClearColor =new Color4(sourceVariable.AsVector().GetVector());
        }
    }
}
