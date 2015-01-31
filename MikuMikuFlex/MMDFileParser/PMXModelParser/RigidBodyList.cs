using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class RigidBodyList
    {
        internal static RigidBodyList GetRigidBodyList(Stream fs, Header header)
        {
            RigidBodyList data=new RigidBodyList();
            data.RigidBodies=new List<RigidBodyData>();
            data.RigidBodyCount = ParserHelper.getInt(fs);
            for (int i = 0; i < data.RigidBodyCount; i++)
            {
                data.RigidBodies.Add(RigidBodyData.GetRigidBodyData(fs,header));
            }
            return data;
        }

        public int RigidBodyCount { get; private set; }

        public List<RigidBodyData> RigidBodies { get; private set; } 
    }
}
