using System;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;

namespace MMF.MME.VariableSubscriber.TextureSubscriber
{
    public class RenderDepthStencilTargetSubscriber:SubscriberBase,IDisposable
    {
        private DepthStencilView depthStencilView;

        private ShaderResourceView shaderResource;

        private Texture2D depthStencilTexture;

        public override string Semantics
        {
            get { return "RENDERDEPTHSTENCILTARGET"; }
        }

        public override VariableType[] Types
        {
            get { return new []{VariableType.Texture2D,VariableType.Texture}; }
        }

        public override SubscriberBase GetSubscriberInstance(EffectVariable variable, RenderContext context, MMEEffectManager effectManager, int semanticIndex)
        {
            RenderDepthStencilTargetSubscriber subscriber=new RenderDepthStencilTargetSubscriber();
            int width, height,depth, mip;
            Format format;
            TextureAnnotationParser.GetBasicTextureAnnotations(variable,context, Format.D24_UNorm_S8_UInt, new Vector2(1f, 1f),false,out width,out height,out depth,out mip,out format);
            Texture2DDescription tex2DDesc = new Texture2DDescription()
            {
                ArraySize = 1,
                BindFlags = BindFlags.DepthStencil | BindFlags.ShaderResource,
                CpuAccessFlags = CpuAccessFlags.None,
                Format = format,
                Height = height,
                Width = width,
                MipLevels = mip,
                OptionFlags = ResourceOptionFlags.None,
                SampleDescription = new SampleDescription(1, 0),
                Usage = ResourceUsage.Default
            };
            subscriber.depthStencilTexture=new Texture2D(context.DeviceManager.Device,tex2DDesc);
            subscriber.depthStencilView=new DepthStencilView(context.DeviceManager.Device,subscriber.depthStencilTexture);
            subscriber.shaderResource=new ShaderResourceView(context.DeviceManager.Device,subscriber.depthStencilTexture);
            effectManager.RenderDepthStencilTargets.Add(variable.Description.Name,subscriber.depthStencilView);
            return subscriber;
        }

        public override void Subscribe(EffectVariable subscribeTo, SubscribeArgument variable)
        {
            subscribeTo.AsResource().SetResource(shaderResource);
        }

        public void Dispose()
        {
            if (this.depthStencilTexture != null && this.depthStencilTexture.Disposed)
                this.depthStencilTexture.Dispose();
            if (this.depthStencilView != null && this.depthStencilView.Disposed)
                this.depthStencilView.Dispose();
            if (this.shaderResource != null && this.shaderResource.Disposed)
                this.shaderResource.Dispose();
        }
    }
}
