using SlimDX;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser.BoneWeight
{
    public class SDEF:BoneWeightBase
    {
        public int Bone1ReferenceIndex;

        public int Bone2ReferenceIndex;

        public float Bone1Weight;

        public Vector3 SDEF_C;

        public Vector3 SDEF_R0;

        public Vector3 SDEF_R1;

    }
}
