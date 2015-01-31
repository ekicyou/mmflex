using System;
using System.Collections.Generic;
using MMF.MME.Script.Function;
using MMF.Model;

namespace MMF.MME.Script
{
    public class ScriptRuntime
    {
        internal static FunctionDictionary ScriptFunctions;

        static ScriptRuntime()
        {
            ScriptFunctions = new FunctionDictionary()
            {
                new RenderColorTargetFunction(),
                new RenderDepthStencilTargetFunction(),
                new ClearSetColorFunction(),
                new ClearSetDepthFunction(),
                new ClearFunction(),
                new PassFunction(),
                new DrawFunction(),
                new LoopByCountFunction(),
                new LoopEndFunction(),
                new LoopGetIndexFunction()
            };
        }

        private ScriptRuntime(string script,RenderContext context,MMEEffectManager manager,MMEEffectTechnique technique,MMEEffectPass pass)
        {
            ScriptCode = script;
            Parse(context,manager,technique,pass);
        }

        public ScriptRuntime(string script, RenderContext context, MMEEffectManager manager,
            MMEEffectTechnique technique):this(script,context,manager,technique,null)
        {     
        }

        public ScriptRuntime(string script, RenderContext context, MMEEffectManager manager,
    MMEEffectPass pass)
            : this(script, context, manager, null,pass)
        {
        }

        public string ScriptCode { get; private set; }

        internal List<FunctionBase> ParsedExecuters;

        public int CurrentExecuter;

        public Stack<int> LoopBegins=new Stack<int>();

        public Stack<int> LoopCounts=new Stack<int>();
 
        public Stack<int> LoopEndCount=new Stack<int>(); 

        private void Parse(RenderContext context, MMEEffectManager manager, MMEEffectTechnique technique, MMEEffectPass pass)
        {
            ParsedExecuters=new List<FunctionBase>();
            if(string.IsNullOrWhiteSpace(ScriptCode))return;
            string[] splittedFunctions = ScriptCode.Split(';');//;で分割し1命令ごとにする
            foreach (string function in splittedFunctions)
            {
                if(string.IsNullOrWhiteSpace(function))continue;
                int index = 0;
                string[] segments = function.Split('=');//=で分割し、関数名と引数を分ける
                if(segments.Length>2)throw new InvalidMMEEffectShaderException("スクリプト中の=の数が多すぎます。");
                char lastCharacter = segments[0][segments[0].Length - 1];
                if (char.IsNumber(lastCharacter))
                {
                    segments[0]=segments[0].Remove(segments[0].Length - 1);
                    index = int.Parse(lastCharacter.ToString());
                }
                if (ScriptFunctions.ContainsKey(segments[0]))
                {
                    ParsedExecuters.Add(ScriptFunctions[segments[0]].GetExecuterInstance(index,segments[1],context,this,manager, technique, pass));
                }
            }
        }

        public void Execute(Action<ISubset> drawAction,ISubset ipmxSubset)
        {
            for (CurrentExecuter=0; CurrentExecuter < ParsedExecuters.Count;ParsedExecuters[CurrentExecuter].Increment(this))
            {
                ParsedExecuters[CurrentExecuter].Execute(ipmxSubset,drawAction);
            }
        }

    }
}
