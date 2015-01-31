using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class VertexList
    {
        public int VertexCount { get; private set; }
        public VertexData[] Vertexes { get; private set; }

        internal static VertexList getVertexList(FileStream fs,Header header)
        {
            VertexList list = new VertexList();
            list.VertexCount = ParserHelper.getInt(fs);
            list.Vertexes = new VertexData[list.VertexCount];
            for (int i = 0; i < list.VertexCount; i++)
            {
                list.Vertexes[i]=(VertexData.getVertex(fs, header));
            }
            return list;
        }
    }
}
