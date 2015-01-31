using System;
using System.Collections.Generic;
using MMF.MME;

namespace MMF.Model
{
    /// <summary>
    ///     レンダリングの方法を引き受けるインターフェース
    /// </summary>
    public interface ISubsetManager : IDisposable
    {
        /// <summary>
        /// サブセットマネージャを初期化し、サブセット分割を実行します。
        /// </summary>
        /// <param name="context">デバイス</param>
        /// <param name="effect"></param>
        /// <param name="subresourceManager"></param>
        /// <param name="ToonManager"></param>
        void Initialze(RenderContext context, MMEEffectManager effect, ISubresourceLoader subresourceManager,
            IToonTextureManager ToonManager);

        void ResetEffect(MMEEffectManager effect);

        void DrawAll();

        void DrawEdges();

        void DrawGroundShadow();

        int SubsetCount { get; }

        List<ISubset> Subsets { get;}
    }
}