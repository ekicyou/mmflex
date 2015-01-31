using System;
using MMF.Model;

namespace MMF.MME.Script.Function
{
    internal class PassFunction:FunctionBase
    {
        public override string FunctionName
        {
            get { return "Pass"; }
        }

        private MMEEffectPass targetPass;

        public override FunctionBase GetExecuterInstance(int index, string value, RenderContext context, ScriptRuntime runtime,
            MMEEffectManager manager, MMEEffectTechnique technique, MMEEffectPass pass)
        {
            PassFunction func=new PassFunction();
            if(technique==null)throw new InvalidMMEEffectShaderException(string.Format("スクリプト内でPassを利用できるのはテクニックに適用されたスクリプトのみです。パスのスクリプトでは実行できません。"));
            if(!technique.Passes.ContainsKey(value))throw new InvalidMMEEffectShaderException(string.Format("スクリプトで指定されたテクニック中では指定されたパス\"{0}\"は見つかりませんでした。(スペルミス?)",value));
            func.targetPass = technique.Passes[value];
            return func;
        }

        public override void Execute(ISubset ipmxSubset, Action<ISubset> drawAction)
        {
            targetPass.Execute(drawAction, ipmxSubset);
        }
    }
}
