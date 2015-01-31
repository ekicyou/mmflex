using System;
using SlimDX;

namespace MMF.Model
{
    /// <summary>
    /// 描画可能リソースのインターフェース
    /// </summary>
    public interface IDrawable : IDisposable
    {

        /// <summary>
        ///     表示されているかどうかを判定する変数
        /// </summary>
        bool Visibility { get; set; }

        /// <summary>
        /// ファイル名を格納する変数
        /// </summary>
        string FileName { get; }

        /// <summary>
        /// サブセットの数
        /// </summary>
        int SubsetCount { get; }

        /// <summary>
        /// 頂点数
        /// </summary>
        int VertexCount { get; }

        /// <summary>
        ///     モデルを動かす際に使用するクラス
        /// </summary>
        ITransformer Transformer { get; }

        /// <summary>
        ///     モデルを描画するときに呼び出します。
        /// </summary>
        void Draw();

        /// <summary>
        ///     モデルを更新するときに呼び出します
        /// </summary>
        void Update();

        /// <summary>
        /// セルフシャドウ色
        /// </summary>
        Vector4 SelfShadowColor { get; set; }

        /// <summary>
        /// 地面影色
        /// </summary>
        Vector4 GroundShadowColor { get; set; }
    }
}