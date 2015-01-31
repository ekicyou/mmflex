using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class SurfaceData
    {
        public uint p;

        public uint q;

        public uint r; 

        public SurfaceData(uint p, uint q, uint r)
        {
            this.p = p; this.q = q; this.r = r;
        }
    }
}
