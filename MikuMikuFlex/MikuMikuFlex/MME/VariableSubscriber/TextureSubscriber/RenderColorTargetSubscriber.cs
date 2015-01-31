using System;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;

namespace MMF.MME.VariableSubscriber.TextureSubscriber
{
    internal class RenderColorTargetSubscriber:SubscriberBase,IDisposable
    {
        public override string Semantics
        {
            get { return  "RENDERCOLORTARGET";}
        }

        public override VariableType[] Types
        {
            get { return new[] {VariableType.Texture2D, VariableType.Texture}; }
        }

        private RenderTargetView renderTarget;

        private ShaderResourceView shaderResource;

        private Texture2D renderTexture;

        private string variableName;

        public override SubscriberBase GetSubscriberInstance(EffectVariable variable, RenderContext context, MMEEffectManager effectManager, int semanticIndex)
        {
            RenderColorTargetSubscriber subscriber=new RenderColorTargetSubscriber();
            variableName = variable.Description.Name;
            int width, height,depth, mip;
            Format format;
            TextureAnnotationParser.GetBasicTextureAnnotations(variable,context, Format.R8G8B8A8_UNorm, new Vector2(1f, 1f),false,out width,out height,out depth,out mip,out format);
            if (depth != -1)
            {
                throw new InvalidMMEEffectShaderException(string.Format("RENDERCOLORTARGETの型はTexture2Dである必要があるためアノテーション「int depth」は指定できません。"));
            }
            Texture2DDescription tex2DDesc = new Texture2DDescription()
            {
                ArraySize = 1,
                BindFlags = BindFlags.RenderTarget | BindFlags.ShaderResource,
                CpuAccessFlags = CpuAccessFlags.None,
                Format = format,
                Height = height,
                Width = width,
                MipLevels = mip,
                OptionFlags = ResourceOptionFlags.None,
                SampleDescription = new SampleDescription(1, 0),
                Usage = ResourceUsage.Default
            };
            subscriber.renderTexture=new Texture2D(context.DeviceManager.Device,tex2DDesc);
           subscriber.renderTarget=new RenderTargetView(context.DeviceManager.Device,subscriber.renderTexture);
            subscriber.shaderResource=new ShaderResourceView(context.DeviceManager.Device,subscriber.renderTexture);
           effectManager.RenderColorTargetViewes.Add(variableName,subscriber.renderTarget);
            return subscriber;
        }

        public override void Subscribe(EffectVariable subscribeTo, SubscribeArgument variable)
        {
            subscribeTo.AsResource().SetResource(shaderResource);
        }

        public void Dispose()
        {
            if(shaderResource!=null&&!shaderResource.Disposed)shaderResource.Dispose();
            if (renderTarget != null && !renderTarget.Disposed) renderTarget.Dispose();
            if(renderTexture!=null&&!renderTexture.Disposed)renderTexture.Dispose();
        }
    }
}
