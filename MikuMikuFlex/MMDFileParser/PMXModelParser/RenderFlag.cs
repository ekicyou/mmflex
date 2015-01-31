using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    [Flags]
    public enum RenderFlag
    {
        CullNone=0x01
       ,GroundShadow=0x02
        ,RenderToZPlot=0x04
        ,RenderSelfShadow=0x08
        ,RenderEdge=0x10
    }
}
