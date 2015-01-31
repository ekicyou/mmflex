using SlimDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class SurfaceList
    {
        public List<SurfaceData> Surfaces { get; private set; }

        public int SurfaceCount { get; private set; }

        internal static SurfaceList getSurfaceList(FileStream fs,Header header)
        {
            SurfaceList list=new SurfaceList();
            list.SurfaceCount=ParserHelper.getInt(fs);
            list.Surfaces=new List<SurfaceData>();
            for(int i=0;i<list.SurfaceCount/3;i++)list.Surfaces.Add(new SurfaceData(ParserHelper.getVertexIndex(fs,header.VertexIndexSize),ParserHelper.getVertexIndex(fs,header.VertexIndexSize),ParserHelper.getVertexIndex(fs,header.VertexIndexSize)));
            return list;
        }
    }
}
