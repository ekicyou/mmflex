using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class TextureList
    {
        public int TextureCount { get; private set; }

        public List<String> TexturePathes { get; private set; }

        internal static　TextureList getTextureList(FileStream fs,Header header)
        {
            TextureList texturelist=new TextureList();
            texturelist.TexturePathes = new List<string>();
            texturelist.TextureCount=ParserHelper.getInt(fs);
            for(int i=0;i<texturelist.TextureCount;i++)
            {
                texturelist.TexturePathes.Add(ParserHelper.getTextBuf(fs,header.Encode));
            }
            return texturelist;
        }
    }
}
