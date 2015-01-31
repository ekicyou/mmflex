using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SlimDX;

namespace MMDFileParser.PMXModelParser.JointParam
{
    public class ConeTwistJointParam: JointParamBase
    {
        internal override void getJointParam(Stream fs, Header header)
        {
            RigidBodyAIndex = ParserHelper.getIndex(fs, header.RigidBodyIndexSize);
            RigidBodyBIndex = ParserHelper.getIndex(fs, header.RigidBodyIndexSize);
            Position = ParserHelper.getFloat3(fs);
            Rotation = ParserHelper.getFloat3(fs);
            Vector3 moveLimitationMin=ParserHelper.getFloat3(fs);
            Vector3 moveLimitationMax=ParserHelper.getFloat3(fs);
            Damping = moveLimitationMin.X;
            FixThresh = moveLimitationMax.X;
            MoterEnabled = Math.Abs(moveLimitationMin.Z - 1) <0.3;//floatなので誤差防止のため(こんなに大きくいらないけど。)
            if (MoterEnabled) MaxMotorImpluse = moveLimitationMax.Z;
            Vector3 rotationLimitationMin = ParserHelper.getFloat3(fs);
            SwingSpan1 = rotationLimitationMin.Z;
            SwingSpan2 = rotationLimitationMin.Y;
            TwistSpan = rotationLimitationMin.X;
            ParserHelper.getFloat3(fs);
            Vector3 springMoveCoefficient = ParserHelper.getFloat3(fs);
            SoftNess = springMoveCoefficient.X;
            BiasFactor = springMoveCoefficient.Y;
            RelaxationFactor = springMoveCoefficient.Z;
            ParserHelper.getFloat3(fs);

        }

        public int RigidBodyAIndex { get; private set; }

        public int RigidBodyBIndex { get; private set; }

        public Vector3 Position { get; private set; }

        public Vector3 Rotation { get; private set; }

        public float SwingSpan1 { get; private set; }

        public float SwingSpan2 { get; private set; }

        public float TwistSpan { get; private set; }

        public float SoftNess { get; private set; }

        public float BiasFactor { get; private set; }

        public float RelaxationFactor { get; private set; }

        public float Damping { get; private set; }

        public float FixThresh { get; private set; }

        public bool MoterEnabled { get;private set; }

        public float MaxMotorImpluse { get; private set; }
    }
}
