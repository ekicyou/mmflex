using SlimDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser.MorphOffset
{
    public class VertexMorphOffset:MorphOffsetBase
    {
        internal static VertexMorphOffset getVertexMorph(FileStream fs, Header header)
        {
            VertexMorphOffset vm = new VertexMorphOffset();
            vm.type = MorphType.Vertex;
            vm.VertexIndex = ParserHelper.getVertexIndex(fs, header.VertexIndexSize);
            vm.PositionOffset = ParserHelper.getFloat3(fs);
            return vm;
        }

        public uint VertexIndex { get; private set; }

        public Vector3 PositionOffset { get; private set; }

    }
}
