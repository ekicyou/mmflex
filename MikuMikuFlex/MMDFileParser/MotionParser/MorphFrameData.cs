using MMDFileParser;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.MotionParser
{
    public class MorphFrameData: IFrameData
    {
        internal static MorphFrameData getMorphFrame(Stream fs)
        {
            var morphFrameData = new MorphFrameData();
            morphFrameData.Name = ParserHelper.getShift_JISString(fs, 15);
            morphFrameData.FrameNumber = ParserHelper.getDWORD(fs);
            morphFrameData.MorphValue = ParserHelper.getFloat(fs);
            return morphFrameData;
        }

        public String Name;

        public float MorphValue;

        public uint FrameNumber { get; private set; }

        public int CompareTo(Object x)
        {
            return (int)FrameNumber - (int)((IFrameData)x).FrameNumber;
        }

    }
}
