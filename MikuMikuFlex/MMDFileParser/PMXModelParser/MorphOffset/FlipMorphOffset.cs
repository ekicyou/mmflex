using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser.MorphOffset
{
    public class FlipMorphOffset:MorphOffsetBase
    {
        internal static FlipMorphOffset getFlipMorph(FileStream fs, Header header)
        {
            FlipMorphOffset fm = new FlipMorphOffset();
            fm.type = MorphType.Flip;
            fm.MorphIndex = ParserHelper.getIndex(fs,header.MorphIndexSize);
            fm.MorphValue = ParserHelper.getFloat(fs);
            return fm;
        }

        public int MorphIndex { get; private set; }

        public float MorphValue { get; private set; }
    }
}
