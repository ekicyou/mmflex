using System;
using MMF.Model;

namespace MMF.MME.Script.Function
{
    internal class LoopEndFunction:FunctionBase
    {
        public override string FunctionName
        {
            get { return "LoopEnd"; }
        }

        public override FunctionBase GetExecuterInstance(int index, string value, RenderContext context, ScriptRuntime runtime,
            MMEEffectManager manager, MMEEffectTechnique technique, MMEEffectPass pass)
        {
            LoopEndFunction func=new LoopEndFunction();
            return func;
        }

        public override void Execute(ISubset ipmxSubset, Action<ISubset> drawAction)
        {

        }

        public override void Increment(ScriptRuntime runtime)
        {
            int loopCount = runtime.LoopEndCount.Pop();
            int count = runtime.LoopCounts.Pop();
            int begin = runtime.LoopBegins.Pop();
            if (count < loopCount)
            {//継続
                runtime.CurrentExecuter = begin+1;//最初の部分+1しておく
                runtime.LoopCounts.Push(count+1);
                runtime.LoopEndCount.Push(loopCount);
                runtime.LoopBegins.Push(begin);
            }
            else
            {//終了
                runtime.CurrentExecuter++;
            }
        }
    }
}
