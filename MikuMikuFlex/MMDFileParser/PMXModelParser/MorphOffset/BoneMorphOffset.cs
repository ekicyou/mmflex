using SlimDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser.MorphOffset
{
    public class BoneMorphOffset:MorphOffsetBase
    {
        internal static BoneMorphOffset getBoneMorph(FileStream fs, Header header)
        {
            BoneMorphOffset bm = new BoneMorphOffset();
            bm.type = MorphType.Bone;
            bm.BoneIndex = ParserHelper.getIndex(fs, header.BoneIndexSize);
            bm.QuantityOfMoving = ParserHelper.getFloat3(fs);
            bm.QuantityOfRotating = ParserHelper.getFloat4(fs);
            return bm;
        }

        public int BoneIndex { get; private set; }

        public Vector3 QuantityOfMoving { get; private set; }

        public Vector4 QuantityOfRotating { get; private set; }
    }
}
