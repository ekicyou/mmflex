using System;
using MMF.Model;
using SlimDX.Direct3D11;

namespace MMF.MME.Script.Function
{
    internal class DrawFunction:FunctionBase
    {
        public override string FunctionName
        {
            get { return "Draw"; }
        }

        private MMEEffectPass targetPass;

        private DeviceContext context;

        private bool isDrawGeometry;

        public override FunctionBase GetExecuterInstance(int index, string value, RenderContext context, ScriptRuntime runtime,
            MMEEffectManager manager, MMEEffectTechnique technique, MMEEffectPass pass)
        {
            DrawFunction func=new DrawFunction();
            if(pass==null) throw new InvalidMMEEffectShaderException(string.Format("Drawはテクニックのスクリプトに関して適用できません。"));
            func.targetPass = pass;
            func.context = context.DeviceManager.Context;
            switch (value)
            {
                case "Geometry":
                    func.isDrawGeometry = true;
                    break;
                case "Buffer":
                    func.isDrawGeometry = false;
                    break;
                default:
                    throw new InvalidMMEEffectShaderException(string.Format("Draw={0}が指定されましたが、Drawに指定できるのはGeometryまたはBufferです。",value));
            }
            if(func.isDrawGeometry&&manager.EffectInfo.ScriptClass==ScriptClass.Scene)throw new InvalidMMEEffectShaderException("Draw=Geometryが指定されましたが、STANDARDGLOBALのScriptClassに\"scene\"を指定している場合、これはできません。");
            if(!func.isDrawGeometry&&manager.EffectInfo.ScriptClass==ScriptClass.Object)throw new InvalidMMEEffectShaderException("Draw=Bufferが指定されましたが、STANDARDGLOBALのScriptClassに\"object\"を指定している場合、これはできません。");
            return func;
        }

        public override void Execute(ISubset ipmxSubset,Action<ISubset> drawAction)
        {
            if (isDrawGeometry)
            {
                targetPass.Pass.Apply(context);
                drawAction(ipmxSubset);
            }
            else
            {
                //TODO Draw=Bufferの場合の処理
            }
        }
    }
}
