using SlimDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser.MorphOffset
{
    public class ImpulseMorphOffset:MorphOffsetBase
    {
        internal static ImpulseMorphOffset getImpulseMorph(FileStream fs, Header header)
        {
            ImpulseMorphOffset im = new ImpulseMorphOffset();
            im.type = MorphType.Impulse;
            im.RigidIndex = ParserHelper.getIndex(fs, header.RigidBodyIndexSize);
            im.LocalFlag = ParserHelper.getByte(fs);
            im.VelocityOfMoving = ParserHelper.getFloat3(fs);
            im.TorqueOfRotating = ParserHelper.getFloat3(fs);
            return im;
        }

        public int RigidIndex { get; private set; }

        public byte LocalFlag { get; private set; }

        public Vector3 VelocityOfMoving { get; private set; }

        public Vector3 TorqueOfRotating { get; private set; }
    }
}
