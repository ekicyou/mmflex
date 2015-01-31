using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMDFileParser.PMXModelParser.JointParam;

namespace MMDFileParser.PMXModelParser
{
    public class JointData
    {
        internal static JointData getJointData(Stream fs, Header header)
        {
            JointData data=new JointData();
            data.JointName = ParserHelper.getTextBuf(fs, header.Encode);
            data.JointName_En = ParserHelper.getTextBuf(fs, header.Encode);
            data.JointType = (JointType)ParserHelper.getByte(fs);
            data.JointParam = JointParamBase.GetJointParamBase(fs, header, data.JointType);
            return data;
        }

        public string JointName { get; private set; }

        public string JointName_En { get; private set; }

        public JointType JointType { get; private set; }

        public JointParamBase JointParam { get; private set; }
    }

    public enum JointType
    {
        Spring6DOF=0,_6DOF=1,P2P=2,ConeTwist=3,Slider=5,Hinge=6
    }
}
