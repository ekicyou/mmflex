using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class MorphList
    {
        internal static MorphList getMorphList(FileStream fs,Header header)
        {
            MorphList ml=new MorphList();
            ml.Morphes = new List<MorphData>();
            ml.MorphCount=ParserHelper.getInt(fs);
            for(int i=0;i<ml.MorphCount;i++)
            {
                ml.Morphes.Add(MorphData.getMorph(fs,header));
            }
            return ml;
        }

        public int MorphCount { get; private set; }

        public List<MorphData> Morphes { get; private set; }
    }
}
