using MMDFileParser;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.MotionParser
{
    public class CameraFrameList
    {
        internal static CameraFrameList getCrameraFrameList(Stream fs)
        {
            CameraFrameList cfl = new CameraFrameList();
            if (fs == null || fs.Position >= fs.Length)
            {
                cfl.CameraFrameCount = 0;
                return cfl;
            }
            try
            {
                cfl.CameraFrameCount = ParserHelper.getDWORD(fs);
                for (int i = 0; i < cfl.CameraFrameCount; i++) cfl.CameraFrames.Add(CameraFrameData.getCameraFrame(fs));
            }catch(Exception e)
            {
                cfl.CameraFrameCount = (uint) cfl.CameraFrames.Count;
                System.Diagnostics.Debug.WriteLine(e.StackTrace + e.Message);
                return cfl;
            }
            
            return cfl;
        }

        public uint CameraFrameCount;

        public List<CameraFrameData> CameraFrames = new List<CameraFrameData>();
    }
}
