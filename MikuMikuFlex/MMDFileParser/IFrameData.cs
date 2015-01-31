using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser
{
    public interface IFrameData: IComparable
    {
        uint FrameNumber { get; }
    }
}
