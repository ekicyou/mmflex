using System;
using MMDFileParser.PMXModelParser;
using MMF.MME.VariableSubscriber.MaterialSubscriber;
using SlimDX.Direct3D11;

namespace MMF.Model.PMX
{
    /// <summary>
    ///     描画単位
    /// </summary>
    public class PMXSubset : IDisposable, IPMXSubset
    {
        public MaterialInfo MaterialInfo { get; private set; }

        public int SubsetId { get; private set; }

        public PMXSubset(IDrawable drawable,MaterialData data,int subsetId)
        {
            Drawable = drawable;
            MaterialInfo = MaterialInfo.FromMaterialData(Drawable, data);
            SubsetId = subsetId;
        }

        /// <summary>
        ///     インデックスバッファにおける、このリソースの開始点
        /// </summary>
        public int StartIndex { get; set; }


        /// <summary>
        ///     このリソースに含まれる頂点数
        /// </summary>
        public int VertexCount { get; set; }

        public void Dispose()
        {
            if(MaterialInfo!=null)MaterialInfo.Dispose();
        }

        public IDrawable Drawable { get; set; }

        public bool DoCulling { get; set; }

        public void Draw(Device device)
        {
            device.ImmediateContext.DrawIndexed(3 * VertexCount, StartIndex, 0);
        }
    }
}