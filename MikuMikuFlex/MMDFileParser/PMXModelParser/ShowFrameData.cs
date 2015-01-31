using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class ShowFrameData
    {
        internal static ShowFrameData getShowFrameData(Stream fs,Header header)
        {
            ShowFrameData data=new ShowFrameData();
            data.FrameName = ParserHelper.getTextBuf(fs, header.Encode);
            data.FrameName_En = ParserHelper.getTextBuf(fs, header.Encode);
            data.IsSpecialFrame = ParserHelper.getByte(fs) == 1;
            data.ElementCount = ParserHelper.getInt(fs);
            data.FrameElements=new List<FrameElementData>();
            for (int i = 0; i < data.ElementCount; i++)
            {
                data.FrameElements.Add(FrameElementData.GetFrameElementData(fs,header));
            }
            return data;
        }

        public string FrameName { get; private set; }

        public string FrameName_En { get; private set; }

        public bool IsSpecialFrame { get; private set; }

        public int ElementCount { get; private set; }

        public List<FrameElementData> FrameElements { get; private set; } 
    }
}
