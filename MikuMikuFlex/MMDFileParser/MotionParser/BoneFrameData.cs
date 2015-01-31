using MMDFileParser;
using SlimDX;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.MotionParser
{
    public class BoneFrameData: IFrameData
    {
        internal static BoneFrameData getBoneFrame(Stream fs)
        {
            BoneFrameData boneFrame = new BoneFrameData();
            boneFrame.BoneName = ParserHelper.getShift_JISString(fs, 15);
            boneFrame.FrameNumber = ParserHelper.getDWORD(fs);
            boneFrame.BonePosition = ParserHelper.getFloat3(fs);
            boneFrame.BoneRotatingQuaternion = ParserHelper.getQuaternion(fs);
            //保管データの読み込み
            boneFrame.Interpolation = new byte[4][][];
            for (int i = 0; i < 4; i++)
            {
                boneFrame.Interpolation[i] = new byte[4][];
                for (int j = 0; j < 4; j++)
                {
                    boneFrame.Interpolation[i][j]=new byte[4];
                }
            }
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    for (int k = 0; k < 4; k++)
                        boneFrame.Interpolation[i][j][k] = ParserHelper.getByte(fs);

            boneFrame.Curves = new BezierCurve[4];
            for (int i = 0; i < boneFrame.Curves.Length; i++)
            {
                BezierCurve curve = new BezierCurve();
                curve.v1 = new Vector2((float)boneFrame.Interpolation[0][0][i] / 128f, (float)boneFrame.Interpolation[0][1][i] / 128f);
                curve.v2 = new Vector2((float)boneFrame.Interpolation[0][2][i] / 128f, (float)boneFrame.Interpolation[0][3][i] / 128f);
                boneFrame.Curves[i] = curve;
            }
            return boneFrame;
        }

        public String BoneName;

        public Vector3 BonePosition;

        public Quaternion BoneRotatingQuaternion;

        public byte[][][] Interpolation;

        public BezierCurve[] Curves;

        public uint FrameNumber { get; private set; }

        public int CompareTo(Object x)
        {
            return (int)FrameNumber - (int)((IFrameData)x).FrameNumber;
        }

    }
}
