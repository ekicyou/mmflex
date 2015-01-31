using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser.MorphOffset
{
    public class GroupMorphOffset:MorphOffsetBase
    {
        internal static GroupMorphOffset getGroupMorph(FileStream fs, Header header)
        {
            GroupMorphOffset gm = new GroupMorphOffset();
            gm.type = MorphType.Group;
            gm.MorphIndex = ParserHelper.getIndex(fs, header.MorphIndexSize);
            gm.MorphRatio = ParserHelper.getFloat(fs);
            return gm;
        }

        public int MorphIndex { get; private set; }

        public float MorphRatio { get; private set; }
    }
}
