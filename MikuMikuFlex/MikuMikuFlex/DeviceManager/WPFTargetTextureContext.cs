using MMF.Matricies;
using SlimDX.Direct3D11;
using SlimDX.DXGI;
using Size = System.Drawing.Size;

namespace MMF.DeviceManager
{
    public class WPFTargetTextureContext:TextureTargetContext
    {
        public WPFTargetTextureContext(RenderContext context, Size size, SampleDescription sampleDesc) : base(context, size, sampleDesc)
        {
        }

        public WPFTargetTextureContext(RenderContext context, MatrixManager matrixManager, Size size, SampleDescription sampleDesc) : base(context, matrixManager, size, sampleDesc)
        {
        }

        protected override Texture2DDescription getRenderTargetTexture2DDescription()
        {
            Texture2DDescription colordesc = new Texture2DDescription();
            colordesc.BindFlags = BindFlags.RenderTarget | BindFlags.ShaderResource;
            colordesc.Format = Format.B8G8R8A8_UNorm;
            colordesc.Width = Size.Width;
            colordesc.Height = Size.Height;
            colordesc.MipLevels = 1;
            colordesc.SampleDescription = new SampleDescription(1, 0);
            colordesc.Usage = ResourceUsage.Default;
            colordesc.OptionFlags = ResourceOptionFlags.Shared;
            colordesc.CpuAccessFlags = CpuAccessFlags.None;
            colordesc.ArraySize = 1;
            return colordesc;
        }
    }
}
