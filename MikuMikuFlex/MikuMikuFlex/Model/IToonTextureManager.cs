using System;
using SlimDX.Direct3D11;

namespace MMF.Model
{
    public interface IToonTextureManager : IDisposable
    {
        /// <summary>
        ///     このアバターのトゥーンの配列
        /// </summary>
        ShaderResourceView[] ResourceViews { get; }

        /// <summary>
        ///     初期化メソッド
        /// </summary>
        /// <param name="context"></param>
        /// <param name="subresourceManager">対象のサブリソースマネージャ</param>
        void Initialize(RenderContext context, ISubresourceLoader subresourceManager);

        /// <summary>
        ///     トゥーンを読み込む
        /// </summary>
        /// <param name="path">Texture名</param>
        /// <returns></returns>
        int LoadToon(string path);
    }
}