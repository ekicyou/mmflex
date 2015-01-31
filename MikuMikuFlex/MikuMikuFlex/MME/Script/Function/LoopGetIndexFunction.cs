using System;
using MMF.Model;
using SlimDX.Direct3D11;

namespace MMF.MME.Script.Function
{
    internal class LoopGetIndexFunction:FunctionBase
    {
        public override string FunctionName
        {
            get { return "LoopGetIndex"; }
        }

        private EffectVariable targetVariable;

        private ScriptRuntime runtime;

        public override FunctionBase GetExecuterInstance(int index, string value, RenderContext context, ScriptRuntime runtime,
            MMEEffectManager manager, MMEEffectTechnique technique, MMEEffectPass pass)
        {
            LoopGetIndexFunction func=new LoopGetIndexFunction();
            func.targetVariable = manager.EffectFile.GetVariableByName(value);
            func.runtime = runtime;
            return func;
        }

        public override void Execute(ISubset ipmxSubset, Action<ISubset> drawAction)
        {
            int count = runtime.LoopCounts.Pop();
            targetVariable.AsScalar().Set(count);
            runtime.LoopCounts.Push(count);
        }
    }
}
