using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SlimDX;

namespace MMDFileParser.PMXModelParser.JointParam
{
    public class SliderJointParam : JointParamBase
    {
        internal override void getJointParam(Stream fs, Header header)
        {
            RigidBodyAIndex = ParserHelper.getIndex(fs, header.RigidBodyIndexSize);
            RigidBodyBIndex = ParserHelper.getIndex(fs, header.RigidBodyIndexSize);
            Position = ParserHelper.getFloat3(fs);
            Rotation = ParserHelper.getFloat3(fs);
            Vector3 moveLimitationMin = ParserHelper.getFloat3(fs);
            Vector3 moveLimitationMax = ParserHelper.getFloat3(fs);
            Vector3 rotationLimitationMin = ParserHelper.getFloat3(fs);
            Vector3 rotationLimitationMax = ParserHelper.getFloat3(fs);
            Vector3 springMoveCoefficient = ParserHelper.getFloat3(fs);
            Vector3 springRotationCoefficient = ParserHelper.getFloat3(fs);
            LowerLinLimit = moveLimitationMin.X;
            UpperLinLimit = moveLimitationMax.X;
            LowerAngLimit = rotationLimitationMin.X;
            UpperAngLimit = rotationLimitationMax.X;
            IsPoweredLinMoter = Math.Abs(springMoveCoefficient.X - 1) < 0.3f;
            if (IsPoweredLinMoter)
            {
                TargetLinMotorVelocity = springMoveCoefficient.Y;
                MaxLinMotorForce = springMoveCoefficient.Z;
            }
            IsPoweredAngMotor = Math.Abs(springRotationCoefficient.X - 1) < 0.3f;
            if (IsPoweredAngMotor)
            {
                TargetAngMotorVelocity = springRotationCoefficient.Y;
                MaxAngMotorForce = springRotationCoefficient.Z;
            }
        }

        public int RigidBodyAIndex { get; private set; }

        public int RigidBodyBIndex { get; private set; }

        public Vector3 Position { get; private set; }

        public Vector3 Rotation { get; private set; }

        public float LowerLinLimit { get; private set; }

        public float UpperLinLimit { get; private set; }

        public float LowerAngLimit { get; private set; }

        public float UpperAngLimit { get; private set; }

        public bool IsPoweredLinMoter { get; private set; }

        public float TargetLinMotorVelocity { get; private set; }

        public float MaxLinMotorForce { get; private set; }

        public bool IsPoweredAngMotor { get; private set; }

        public float TargetAngMotorVelocity { get; private set; }

        public float MaxAngMotorForce { get; private set; }


    }
}
