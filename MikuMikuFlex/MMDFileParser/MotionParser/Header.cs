using MMDFileParser;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.MotionParser
{
    public class Header
    {
        internal static Header getHeader(Stream fs)
        {
            Header h = new Header();
            h.HeaderStr = ParserHelper.getShift_JISString(fs, 30);
            h.ModelName = ParserHelper.getShift_JISString(fs, 20);
            return h;
        }

        public String HeaderStr;

        public String ModelName;
    }
}
