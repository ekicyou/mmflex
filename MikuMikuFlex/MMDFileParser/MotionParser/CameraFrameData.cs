using System.Globalization;
using MMDFileParser;
using SlimDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.MotionParser
{
    public class CameraFrameData:IComparer<CameraFrameData>
    {
        internal static CameraFrameData getCameraFrame(Stream fs)
        {
            CameraFrameData cf = new CameraFrameData();
            cf.FrameNumber = ParserHelper.getDWORD(fs);
            cf.Distance = ParserHelper.getFloat(fs);
            cf.CameraPosition = ParserHelper.getFloat3(fs);
            cf.CameraRotation = ParserHelper.getFloat3(fs);
            cf.CameraRotation.X = -cf.CameraRotation.X;//カメラのX軸回転は逆のため符号を反転しておく
            cf.Interpolation=new byte[6][];
            for (int i = 0; i < 6; i++)
            {
                cf.Interpolation[i] = new byte[4];
            }
            for(int i=0;i<6;i++) for (int j = 0; j < 4; j++) cf.Interpolation[i][j] = ParserHelper.getByte(fs);
            cf.ViewAngle = ParserHelper.getDWORD(fs);
            cf.Perspective = ParserHelper.getByte(fs);
            cf.Curves=new BezierCurve[6];
            for (int i = 0; i < 6; i++)
            {
                BezierCurve curve=new BezierCurve();
                curve.v1=new Vector2(cf.Interpolation[i][0]/128f,cf.Interpolation[i][1]/128f);
                curve.v2=new Vector2(cf.Interpolation[i][2]/128f,cf.Interpolation[i][3]/128f);
                cf.Curves[i] = curve;
            }
            return cf;
        }

        public uint FrameNumber;

        public float Distance;

        public Vector3 CameraPosition;

        public Vector3 CameraRotation;

        public byte[][] Interpolation;

        public BezierCurve[] Curves;

        public uint ViewAngle;

        public byte Perspective;

        
        public int Compare(CameraFrameData x, CameraFrameData y)
        {
            return (int) (x.FrameNumber - y.FrameNumber);
        }
    }
}
