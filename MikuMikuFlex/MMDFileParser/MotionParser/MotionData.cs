using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.MotionParser
{
    public class MotionData
    {
        public static MotionData getMotion(Stream fs)
        {
            MotionData motion = new MotionData();
            motion.header = Header.getHeader(fs);
            motion.boneFrameList = BoneFrameList.getBoneFrameList(fs);
            motion.morphFrameList = MorphFrameList.getFraceFrameList(fs);
            motion.CameraFrames = CameraFrameList.getCrameraFrameList(fs);
            motion.LightFrames = LightFrameList.getLightFrameList(fs);
            return motion;
        }
        public Header header;

        public BoneFrameList boneFrameList;

        public MorphFrameList morphFrameList;

        public CameraFrameList CameraFrames;

        public LightFrameList LightFrames;
    }
}
