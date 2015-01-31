using System;
using System.IO;

namespace MMDFileParser.PMXModelParser.JointParam
{
    public abstract class JointParamBase
    {
        internal static JointParamBase GetJointParamBase(Stream fs, Header header, JointType type)
        {
            switch (type)
            {
               case JointType.Spring6DOF:
                    Spring6DofJointParam sp6=new Spring6DofJointParam();
                    sp6.getJointParam(fs, header);
                    return sp6;
                default:
                    throw new NotSupportedException("PMX2.0までしかサポートしてません。");
            }
        }

        internal abstract void getJointParam(Stream fs, Header header);
    }
}