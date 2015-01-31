using System;
using MMF.MME.Script;
using MMF.Model;
using SlimDX.Direct3D11;

namespace MMF.MME
{
    /// <summary>
    ///     エフェクトのパスを管理するクラス
    /// </summary>
    public class MMEEffectPass
    {
        private RenderContext context;

        public MMEEffectPass(RenderContext context,MMEEffectManager manager, EffectPass pass)
        {
            this.context = context;
            Pass = pass;
            EffectVariable commandAnnotation = EffectParseHelper.getAnnotation(pass, "Script", "string");
            Command = commandAnnotation == null ? "" : commandAnnotation.AsString().GetString();
            if (!pass.VertexShaderDescription.Variable.IsValid)
            {
                //TODO この場合標準シェーダーの頂点シェーダを利用する
            }
            if (!pass.PixelShaderDescription.Variable.IsValid)
            {
                //TODO この場合標準シェーダーのピクセルシェーダを利用する
            }
            ScriptRuntime=new ScriptRuntime(Command,context,manager,this);
        }

        public void Execute(Action<ISubset> drawAction,ISubset ipmxSubset)
        {
            if (string.IsNullOrWhiteSpace(ScriptRuntime.ScriptCode))
            {
                Pass.Apply(context.DeviceManager.Context);
                drawAction(ipmxSubset);
            }
            else//スクリプトが存在する場合は処理をスクリプトランタイムに任せる
            {
                ScriptRuntime.Execute(drawAction, ipmxSubset);
            }
        }

        /// <summary>
        ///     コマンドアノテーション
        /// </summary>
        public string Command { get; private set; }

        public ScriptRuntime ScriptRuntime { get; private set; }

        /// <summary>
        ///     描画に利用されるパス
        /// </summary>
        public EffectPass Pass { get; private set; }
    }
}