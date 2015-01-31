using SlimDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser.MorphOffset
{
    public class UVMorphOffset:MorphOffsetBase
    {
        internal static UVMorphOffset getUVMorph(FileStream fs, Header header,MorphType type)
        {
            UVMorphOffset uv = new UVMorphOffset();
            uv.VertexIndex = ParserHelper.getVertexIndex(fs, header.VertexIndexSize);
            uv.UVOffset = ParserHelper.getFloat4(fs);
            uv.type = type;
            return uv;
        }

        public uint VertexIndex { get; private set; }

        public Vector4 UVOffset { get; private set; }
    }
}
