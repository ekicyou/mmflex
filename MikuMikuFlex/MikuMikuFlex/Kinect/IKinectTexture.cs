using System;
using SlimDX.Direct3D11;

namespace MMF.Kinect
{
    public interface IKinectTexture : IDisposable
    {
        Texture2D ColorTexture2D { get; }
        ShaderResourceView ColorTexture2DResourceView { get; }
        void UpdateTexture();
    }
}