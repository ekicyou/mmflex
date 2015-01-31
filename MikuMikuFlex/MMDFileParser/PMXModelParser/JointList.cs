using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class JointList
    {
        internal static JointList getJointList(Stream fs, Header header)
        {
            JointList data=new JointList();
            data.JointCount = ParserHelper.getInt(fs);
            data.Joints=new List<JointData>();
            for (int i = 0; i < data.JointCount; i++)
            {
                data.Joints.Add(JointData.getJointData(fs,header));
            }
            return data;
        }

        public int JointCount { get; private set; }

        public List<JointData> Joints { get; private set; } 
    }
}
