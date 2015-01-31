using System;
using MMF.Matricies;
using MMF.Model;
using SlimDX.Direct3D11;

namespace MMF.Light
{
    /// <summary>
    ///     影を落とすことができるものにつけるインターフェース
    /// </summary>
    public interface IShadowmap : IDisposable
    {
        /// <summary>
        ///     ライトから見た行列
        /// </summary>
        MatrixManager MatriciesFromLight { get; }

        /// <summary>
        ///     レンダーターゲット
        /// </summary>
        RenderTargetView ShadowBufferRenderTarget { get; }

        /// <summary>
        ///     深度ステンシルビュー
        /// </summary>
        DepthStencilView ShadowBufferDepthStencil { get; }

        /// <summary>
        ///     テクスチャ
        /// </summary>
        Texture2D ShadowBufferDepthTexture { get; }

        /// <summary>
        ///     テクスチャから利用した深度のリソース
        /// </summary>
        ShaderResourceView DepthTextureResource { get; }


        /// <summary>
        ///     移動用のTransformer
        /// </summary>
        ITransformer Transformer { get; }
    }
}