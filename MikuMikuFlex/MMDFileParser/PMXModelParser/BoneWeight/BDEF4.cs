using SlimDX;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser.BoneWeight
{
    public class BDEF4:BoneWeightBase
    {
        public int Bone1ReferenceIndex;

        public int Bone2ReferenceIndex;

        public int Bone3ReferenceIndex;

        public int Bone4ReferenceIndex;

        /// <summary>
        /// (x,y,z,w)=(Bone1Weight,Bone2Weight,Bone3Weight,Bone4Weight)
        /// </summary>
        public Vector4 Weights;
    }
}
