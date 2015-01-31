using System.Runtime.InteropServices;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;

namespace MMF.Grid
{
    /// <summary>
    ///     グリッド描画用の入力レイアウト
    /// </summary>
    public struct MeasureGridLayout
    {
        /// <summary>
        ///     入力レイアウト
        /// </summary>
        public static readonly InputElement[] VertexElements =
        {
            new InputElement
            {
                SemanticName = "POSITION",
                Format = Format.R32G32B32_Float
            }
        };

        /// <summary>
        ///     位置
        /// </summary>
        public Vector3 Position;

        /// <summary>
        ///     1要素あたりのサイズ
        /// </summary>
        public static int SizeInBytes
        {
            get { return Marshal.SizeOf(typeof (MeasureGridLayout)); }
        }
    }
}