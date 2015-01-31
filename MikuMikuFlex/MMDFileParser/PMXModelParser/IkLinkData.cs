using SlimDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class IkLinkData
    {
        internal static IkLinkData getIKLink(FileStream fs,Header header)
        {
            IkLinkData ikLinkData=new IkLinkData();
            ikLinkData.LinkBoneIndex=ParserHelper.getIndex(fs,header.BoneIndexSize);
            ikLinkData.isRotateLimited=ParserHelper.getByte(fs)==1?true:false;
            if(ikLinkData.isRotateLimited)
            {
                ikLinkData.MinimumRadian=ParserHelper.getFloat3(fs);
                ikLinkData.MaximumRadian=ParserHelper.getFloat3(fs);
            }
            return ikLinkData;
        }
        public int LinkBoneIndex { get; private set; }
        public bool isRotateLimited { get; private set; }
        public Vector3 MinimumRadian { get; private set; }
        public Vector3 MaximumRadian { get; private set; }
    }
}
