using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SlimDX;

namespace MMDFileParser.PMXModelParser
{
    /// <summary>
    /// 剛体情報を保持するクラス
    /// </summary>
    public class RigidBodyData
    {
        internal static RigidBodyData GetRigidBodyData(Stream fs, Header header)
        {
            RigidBodyData data=new RigidBodyData();
            data.RigidBodyName = ParserHelper.getTextBuf(fs, header.Encode);
            data.RigidBodyName_En = ParserHelper.getTextBuf(fs, header.Encode);
            data.BoneIndex = ParserHelper.getIndex(fs, header.BoneIndexSize);
            data.RigidBodyGroup = ParserHelper.getByte(fs);
            data.UnCollisionGroupFlag = ParserHelper.getUShort(fs);
            data.Shape = (RigidBodyShape)ParserHelper.getByte(fs);
            data.Size = ParserHelper.getFloat3(fs);
            data.Position = ParserHelper.getFloat3(fs);
            data.Rotation = ParserHelper.getFloat3(fs);
            data.Mass = ParserHelper.getFloat(fs);
            data.MoveAttenuation = ParserHelper.getFloat(fs);
            data.RotationAttenuation = ParserHelper.getFloat(fs);
            data.Repulsion = ParserHelper.getFloat(fs);
            data.Friction = ParserHelper.getFloat(fs);
            data.PhysicsCalcType = (PhysicsCalcType) ParserHelper.getByte(fs);
            return data;
        }

        public string RigidBodyName { get; private set; }

        public string RigidBodyName_En { get; private set; }

        public int BoneIndex { get; private set; }

        public byte RigidBodyGroup { get; private set; }

        public ushort UnCollisionGroupFlag { get; private set; }

        public RigidBodyShape Shape { get; private set; }

        public Vector3 Size { get; private set; }

        public Vector3 Position { get; private set; }

        public Vector3 Rotation { get; private set; }

        public float Mass { get; private set; }

        public float MoveAttenuation { get; private set; }

        public float RotationAttenuation { get; private set; }

        public float Repulsion { get;private set; }

        public float Friction { get; private set; }

        public PhysicsCalcType PhysicsCalcType { get; private set; }

    }

    public enum PhysicsCalcType
    {
        Static=0,Dynamic=1,BoneAlignment=2
    }

    public enum RigidBodyShape
    {
        Sphere=0,Box=1,Capsule=2
    }
}
