using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class ShowFrameList
    {
        internal static ShowFrameList getShowFrameList(Stream fs, Header header)
        {
            ShowFrameList data=new ShowFrameList();
            data.ShowFrameCount = ParserHelper.getInt(fs);
            data.ShowFrames=new List<ShowFrameData>();
            for (int i = 0; i < data.ShowFrameCount; i++)
            {
                data.ShowFrames.Add(ShowFrameData.getShowFrameData(fs,header));
            }
            return data;
        }

        public int ShowFrameCount { get; private set; }

        public List<ShowFrameData> ShowFrames { get; private set; } 
    }
}
