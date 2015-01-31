using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Runtime.InteropServices;
using MMF.Sprite;
using NiTEWrapper;
using OpenNIWrapper;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;
using Device = OpenNIWrapper.Device;
using MapFlags = SlimDX.Direct3D11.MapFlags;

namespace MMF.Kinect
{
    public class ColorTexture: IDynamicTexture
    {
        private RenderContext context;
        public Texture2D TextureResource { get; set; }

        public short TrackId
        {
            get { return _trackId; }
            set
            {
                _trackId = value;
                if (_trackId > KinectDevice.CurrentUserTrackerFrameRef.Users.Length) _trackId = 0;
            }
        }

        public ShaderResourceView TextureResourceView { get; private set; }
        private VideoStream videoStream;



        public ColorTexture(RenderContext context,KinectDeviceManager kinectDevice)
        {
            this.KinectDevice = kinectDevice;
            this.context = context;
            Texture2DDescription tex2DDesc = new Texture2DDescription()
            {
                ArraySize = 1,
                BindFlags = BindFlags.ShaderResource,
                CpuAccessFlags = CpuAccessFlags.Write,
                Format = Format.R8G8B8A8_UNorm,
                Height = 480,
                Width = 640,
                MipLevels = 1,
                OptionFlags = ResourceOptionFlags.None,
                SampleDescription = new SampleDescription(1,0),
                Usage =ResourceUsage.Dynamic
            };
            TextureResource=new Texture2D(context.DeviceManager.Device,tex2DDesc);
            TextureResourceView=new ShaderResourceView(context.DeviceManager.Device,TextureResource);
            videoStream = kinectDevice.KinnectDevice.CreateVideoStream(Device.SensorType.COLOR);
            videoStream.Start();
            NeedUpdate = true;
        }

        public KinectDeviceManager KinectDevice
        { get; set; }

        private short _trackId;

        private List<Tuple<SkeletonJoint.JointType,SkeletonJoint.JointType>> drawJoints=new List<Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>>()
        {
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.HEAD, SkeletonJoint.JointType.NECK),
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.NECK, SkeletonJoint.JointType.LEFT_SHOULDER),
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.NECK, SkeletonJoint.JointType.RIGHT_SHOULDER),
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.LEFT_SHOULDER, SkeletonJoint.JointType.LEFT_ELBOW),
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.LEFT_ELBOW, SkeletonJoint.JointType.LEFT_HAND),
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.RIGHT_SHOULDER,SkeletonJoint.JointType.RIGHT_ELBOW),
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.RIGHT_ELBOW,SkeletonJoint.JointType.RIGHT_HAND),
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.NECK,SkeletonJoint.JointType.TORSO),
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.TORSO, SkeletonJoint.JointType.RIGHT_HIP),
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.TORSO, SkeletonJoint.JointType.LEFT_HIP),
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.LEFT_HIP, SkeletonJoint.JointType.LEFT_KNEE),
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.RIGHT_HIP,SkeletonJoint.JointType.RIGHT_KNEE),
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.LEFT_KNEE, SkeletonJoint.JointType.LEFT_FOOT),
            new Tuple<SkeletonJoint.JointType, SkeletonJoint.JointType>(SkeletonJoint.JointType.RIGHT_KNEE, SkeletonJoint.JointType.RIGHT_FOOT)
        }; 

        public void UpdateTexture()
        {
            int width = 640;
            int height = 480;
            DataBox mapSubresource = context.DeviceManager.Context.MapSubresource(TextureResource,0, MapMode.WriteDiscard, MapFlags.None);
            VideoFrameRef vidRef = videoStream.readFrame();
            UserTrackerFrameRef usrRef = KinectDevice.CurrentUserTrackerFrameRef;
            IntPtr intPtr = usrRef.UserMap.Pixels;
            byte[] bits=new byte[width*height*3];
            byte[] ubits=new byte[width*height*2];
            List<byte> drawed=new List<byte>();
            Marshal.Copy(vidRef.Data,bits,0,width*height*3);
            Marshal.Copy(intPtr,ubits,0,width*height*2);
            mapSubresource.Data.Seek(0, SeekOrigin.Begin);

            UserData cursorUser = KinectDevice.CurrentUserTrackerFrameRef.getUserById(KinectDevice.UserCursor);
            var targetSkel = cursorUser.Skeleton;

            for (int i = 0; i < width*height; i++)
            {
                short uid = BitConverter.ToInt16(ubits, i*2);
               
                if (cursorUser.isValid&&uid==KinectDevice.UserCursor)
                {
                    if (targetSkel.State == Skeleton.SkeletonState.CALIBRATING)
                    {
                        drawed.Add(255);
                        drawed.Add(255);
                        drawed.Add(0);
                        drawed.Add(255);
                        continue;
                    }
                    else if (targetSkel.State == Skeleton.SkeletonState.TRACKED)
                    {
                        drawed.Add(0);
                        drawed.Add(255);
                        drawed.Add(0);
                        drawed.Add(255);
                        continue;
                    }
                    else if (targetSkel.State == Skeleton.SkeletonState.NONE)
                    {
                        drawed.Add(0);
                        drawed.Add(255);
                        drawed.Add(255);
                        drawed.Add(255);
                        continue;
                    }
                }
                drawed.Add(bits[i*3]);
                drawed.Add(bits[i * 3+1]);
                drawed.Add(bits[i * 3+2]);
                drawed.Add(255);
            }
            foreach (var trackedUser in KinectDevice.TrackedUsers)
            {
                UserData TargetUser = trackedUser.Value;
                foreach (var drawJoint in drawJoints)
                {
                    SkeletonJoint j1 = TargetUser.Skeleton.getJoint(drawJoint.Item1);
                    SkeletonJoint j2 = TargetUser.Skeleton.getJoint(drawJoint.Item2);
                    PointF p1 =
                        KinectDevice.NiteUserTracker.ConvertJointCoordinatesToDepth(j1.Position);
                    PointF p2 =
                        KinectDevice.NiteUserTracker.ConvertJointCoordinatesToDepth(j2.Position);
                    byte blue = (byte)(255f * (j1.PositionConfidence + j2.PositionConfidence) / 2f);
                    DrawLine((int)p1.X, (int)p1.Y, (int)p2.X, (int)p2.Y, 2, new byte[] { (byte)(255 - blue), 0, blue, 255 }, drawed);
                }
            }

            mapSubresource.Data.WriteRange(drawed.ToArray());
            context.DeviceManager.Context.UnmapSubresource(TextureResource,0);
        }

        public bool NeedUpdate { get; private set; }

        private int PicWideth = 640;
        private int PicHeight = 480;
        /// <summary>
        /// 2点の座標の組と引く線分の幅、色
        /// </summary>
        /// <param name="x1"></param>
        /// <param name="y1"></param>
        /// <param name="x2"></param>
        /// <param name="y2"></param>
        /// <param name="wideth"></param>
        /// <param name="drawed"></param>
        /// <param name="color"></param>
        public void DrawLine(int x1, int y1, int x2, int y2, int wideth, byte[] color, List<byte> drawed)
        {
            if ((x1 < -1) || (PicWideth - 1 < x1) || (x2 < -1) || (PicWideth - 1 < x2)) return;
            if ((y1 < -1) || (PicHeight - 1 < y1) || (y2 < -1) || (PicHeight - 1 < y2)) return;
            if ((x1 == x2) && (y1 == y2)) return;

            if (x1 == x2)
            {
                int maxY = Math.Max(y1, y2);
                int minY = Math.Min(y1, y2);

                for (int i = minY; i < maxY + 1; i++)
                {
                    for (int j = x1 - wideth; j < x1 + wideth + 1; j++)
                    {
                        for (int k = 0; k < 4; k++)
                        {
                            drawed[4 * (PicWideth * i + j) + k] = color[k];
                        }
                    }
                }
                return;
            }

            if (y1 == y2)
            {
                int maxX = Math.Max(x1, x2);
                int minX = Math.Min(x1, x2);

                for (int i = y1 - wideth; i < y1 + wideth + 1; i++)
                {
                    for (int j = minX; j < maxX + 1; j++)
                    {
                        for (int k = 0; k < 4; k++)
                        {
                            drawed[4 * (PicWideth * i + j) + k] = color[k];
                        }
                    }
                }
                return;
            }


            int a = y1 - y2;
            int b = x2 - x1;
            int c = x1 * y2 - x2 * y1;

            int max = Math.Max(x1, x2);
            int min = Math.Min(x1, x2);
            int max2 = Math.Max(y1, y2);
            int min2 = Math.Min(y1, y2);

            float s, t;

            for (int i = min2; i < max2 + 1; i++)
            {
                for (int j = min; j < max + 1; j++)
                {
                    s = Math.Abs((float)(a * j + b * i + c) / a);
                    t = Math.Abs((float)(a * j + b * i + c) / b);
                    if ((s * t) / Math.Sqrt(t * t + s * s) < wideth)
                    {
                        for (int k = 0; k < 4; k++)
                        {
                            drawed[4 * (PicWideth * i + j) + k] = color[k];
                        }
                    }
                }
            }
        }

        public void Dispose()
        {
            TextureResource.Dispose();
            TextureResourceView.Dispose();
        }
    }
}
