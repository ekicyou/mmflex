using MMDFileParser;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.MotionParser
{
    public class BoneFrameList
    {
        internal static BoneFrameList getBoneFrameList(Stream fs)
        {
            BoneFrameList list = new BoneFrameList();
            try
            {
                list.BoneFrameCount = ParserHelper.getDWORD(fs);
            }
            catch (EndOfStreamException eof)
            {
                list.BoneFrameCount = 0;
                return list;
            }
            for (int i = 0; i < list.BoneFrameCount; i++) list.boneFrameDatas.Add(BoneFrameData.getBoneFrame(fs));
            return list;
        }

        public uint BoneFrameCount;

        public List<BoneFrameData> boneFrameDatas = new List<BoneFrameData>();
    }
}
