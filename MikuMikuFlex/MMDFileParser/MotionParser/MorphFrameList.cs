using MMDFileParser;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.MotionParser
{
    public class MorphFrameList
    {
        internal static MorphFrameList getFraceFrameList(Stream fs)
        {
            var morphFrameList = new MorphFrameList();
            try
            {
                morphFrameList.MorphFrameCount = ParserHelper.getDWORD(fs);
            }
            catch
            {
                morphFrameList.MorphFrameCount = 0;
                return morphFrameList;
            }
            for (int i = 0; i < morphFrameList.MorphFrameCount; i++) morphFrameList.morphFrameDatas.Add(MorphFrameData.getMorphFrame(fs));
            return morphFrameList;
        }
        public uint MorphFrameCount;

        public List<MorphFrameData> morphFrameDatas = new List<MorphFrameData>();
    }
}
