using System;
using SlimDX.Direct3D11;

namespace MMF.Sprite
{
    public interface IDynamicTexture : IDisposable
    {
        Texture2D TextureResource { get; }

        ShaderResourceView TextureResourceView { get; }

        void UpdateTexture();

        bool NeedUpdate { get; }
    }
}