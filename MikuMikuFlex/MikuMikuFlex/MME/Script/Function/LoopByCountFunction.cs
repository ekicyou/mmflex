using System;
using MMF.Model;
using SlimDX.Direct3D11;

namespace MMF.MME.Script.Function
{
    internal class LoopByCountFunction:FunctionBase
    {
        public override string FunctionName
        {
            get { return "LoopByCount"; }
        }

        private ScriptRuntime runtime;

        private int loopCount;

        public override FunctionBase GetExecuterInstance(int index, string value, RenderContext context, ScriptRuntime runtime,
            MMEEffectManager manager, MMEEffectTechnique technique, MMEEffectPass pass)
        {
            LoopByCountFunction func=new LoopByCountFunction();
            func.runtime = runtime;
            if(string.IsNullOrWhiteSpace(value))throw new InvalidMMEEffectShaderException("LoopByCount=;は指定できません。int,float,boolいずれかの変数名を伴う必要があります。");
            EffectVariable rawVariable = manager.EffectFile.GetVariableByName(value);
            string typeName = rawVariable.GetVariableType().Description.TypeName.ToLower();
            int loopCount = 0;
            switch (typeName)
            {
                case "bool":
                case "int":
                    loopCount = rawVariable.AsScalar().GetInt();
                    break;
                case "float":
                    loopCount = (int)rawVariable.AsScalar().GetFloat();
                    break;
                default:
                    throw new InvalidMMEEffectShaderException("LoopByCountに指定できる変数の型はfloat,int,boolのいずれかです。");
            }
            func.loopCount = loopCount;
            return func;
        }

        public override void Execute(ISubset ipmxSubset, Action<ISubset> drawAction)
        {
            runtime.LoopBegins.Push(runtime.ParsedExecuters.Count);
            runtime.LoopCounts.Push(0);
            runtime.LoopEndCount.Push(loopCount);
        }
    }
}
