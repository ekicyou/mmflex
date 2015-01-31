using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMDFileParser.MotionParser;
using OpenMMDFormat;

namespace VMD2VME
{
    class FrameComparator:IComparer<BoneFrame>,IComparer<MorphFrame>
    {
        public int Compare(BoneFrame x, BoneFrame y)
        {
            return (int) (x.frameNumber - y.frameNumber);
        }

        public int Compare(MorphFrame x, MorphFrame y)
        {
            return (int) (x.frameNumber - y.frameNumber);
        }
    }
}
