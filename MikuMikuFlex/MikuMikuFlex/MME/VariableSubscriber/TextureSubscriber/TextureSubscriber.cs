using System;
using System.IO;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;
using Resource = SlimDX.Direct3D11.Resource;

namespace MMF.MME.VariableSubscriber.TextureSubscriber
{
    internal class TextureSubscriber:SubscriberBase,System.IDisposable
    {
        private ShaderResourceView resourceView;

        private Resource resourceTexture;

        public override string Semantics
        {
            get { throw new InvalidOperationException("このサブスクライバにはセマンティクスを持ちません"); }
        }

        public override VariableType[] Types
        {
            get { return new[] {VariableType.Texture, VariableType.Texture2D,VariableType.Texture3D,VariableType.TextureCUBE}; }
        }

        public override SubscriberBase GetSubscriberInstance(EffectVariable variable, RenderContext context, MMEEffectManager effectManager, int semanticIndex)
        {
            TextureSubscriber subscriber=new TextureSubscriber();
            int width, height, depth, mip;
            string typeName = variable.GetVariableType().Description.TypeName.ToLower();
            string type;
            Format format;
            TextureAnnotationParser.GetBasicTextureAnnotations(variable, context, Format.B8G8R8A8_UNorm, Vector2.Zero,true,
                out width, out height, out depth, out mip, out format);
            EffectVariable rawTypeVariable = EffectParseHelper.getAnnotation(variable, "ResourceType", "string");
            EffectVariable rawStringVariable = EffectParseHelper.getAnnotation(variable, "ResourceName", "string");
            if (rawTypeVariable != null)
            {

                switch (rawTypeVariable.AsString().GetString().ToLower())
                {
                    case "cube":
                        if (!typeName.Equals("texturecube"))
                        {
                            throw new InvalidMMEEffectShaderException("ResourceTypeにはCubeが指定されていますが、型がtextureCUBEではありません。");
                        }
                        else
                        {
                            type = "texturecube";
                        }
                        break;
                    case "2d":
                        if (!typeName.Equals("texture2d")&&!typeName.Equals("texture"))
                        {
                            throw new InvalidMMEEffectShaderException("ResourceTypeには2Dが指定されていますが、型がtexture2Dもしくはtextureではありません。");
                        }
                        else
                        {
                            type = "texture2d";
                        }
                        break;
                    case "3d":
                        if (!typeName.Equals("texture3d"))
                        {
                            throw new InvalidMMEEffectShaderException("ResourceTypeには3Dが指定されていますが、型がtexture3dではありません。");
                        }
                        else
                        {
                            type = "texture3d";
                        }
                        break;
                    default:
                        throw new InvalidMMEEffectShaderException("認識できないResourceTypeが指定されました。");
                }
            }
            else
            {
                type = typeName;
            }
            if (rawStringVariable != null)
            {
                string resourceName = rawStringVariable.AsString().GetString();
                ImageLoadInformation imgInfo = new ImageLoadInformation();
                Stream stream;
                switch (type)
                {
                    case "texture2d":
                        
                        imgInfo.Width = width;
                        imgInfo.Height = height;
                        imgInfo.MipLevels = mip;
                        imgInfo.Format = format;
                        imgInfo.Usage=ResourceUsage.Default;
                        imgInfo.BindFlags=BindFlags.ShaderResource;
                        imgInfo.CpuAccessFlags=CpuAccessFlags.None;
                        stream = effectManager.SubresourceLoader.getSubresourceByName(resourceName);
                        if(stream!=null)
                        subscriber.resourceTexture=Texture2D.FromStream(context.DeviceManager.Device, stream, (int)stream.Length);
                        break;
                    case "texture3d":
                        imgInfo.Width = width;
                        imgInfo.Height = height;
                        imgInfo.Depth = depth;
                        imgInfo.MipLevels = mip;
                        imgInfo.Format = format;
                        imgInfo.Usage=ResourceUsage.Default;
                        imgInfo.BindFlags=BindFlags.ShaderResource;
                        imgInfo.CpuAccessFlags=CpuAccessFlags.None;
                        stream = effectManager.SubresourceLoader.getSubresourceByName(resourceName);
                        if(stream!=null)
                        subscriber.resourceTexture=Texture3D.FromStream(context.DeviceManager.Device, stream, (int)stream.Length);
                        break;
                    case "texturecube":
                        //TODO CUBEの場合に対応する
                        //imgInfo.Width = width;
                        //imgInfo.Height = height;
                        //imgInfo.Depth = depth;
                        //imgInfo.MipLevels = mip;
                        //imgInfo.Format = format;
                        //imgInfo.Usage=ResourceUsage.Default;
                        //imgInfo.BindFlags=BindFlags.ShaderResource;
                        //imgInfo.CpuAccessFlags=CpuAccessFlags.None;
                        //stream = effectManager.SubresourceLoader.getSubresourceByName(resourceName);
                        //subscriber.resourceTexture=.FromStream(context.DeviceManager.Device, stream, (int)stream.Length);
                        break;
                }
            }
            subscriber.resourceView=new ShaderResourceView(context.DeviceManager.Device,subscriber.resourceTexture);
            return subscriber;
        }

        public override void Subscribe(EffectVariable subscribeTo, SubscribeArgument variable)
        {
            subscribeTo.AsResource().SetResource(resourceView);
        }

        public void Dispose()
        {
            if(this.resourceView != null && !this.resourceView.Disposed) 
                this.resourceView.Dispose();
        }
    }
}
