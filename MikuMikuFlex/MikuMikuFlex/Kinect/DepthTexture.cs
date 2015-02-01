using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using MMF.Sprite;
using OpenNIWrapper;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;
using MapFlags = SlimDX.Direct3D11.MapFlags;

namespace MMF.Kinect
{
    public class DepthTexture:IDynamicTexture
    {
        private RenderContext context;
        public Texture2D TextureResource { get;private set; }
        public ShaderResourceView TextureResourceView { get; private set; }

        /// <summary>
        /// 最大の距離、これ以上遠い場合真っ白になる
        /// </summary>
        public int MaxDistance { get; set; }

        private VideoStream videoStream;

        public DepthTexture(RenderContext context,int maxDistance,KinectDeviceManager device)
        {
            this.context = context;
            this.MaxDistance = maxDistance;
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
            videoStream = device.KinnectDevice.CreateVideoStream(OpenNIWrapper.Device.SensorType.Depth);
            videoStream.Start();
            TextureResourceView = new ShaderResourceView(context.DeviceManager.Device, TextureResource);
            NeedUpdate = true;
        }

        public void UpdateTexture()
        {
            int width = 640;
            int height = 480;
            DataBox mapSubresource = context.DeviceManager.Context.MapSubresource(TextureResource,0, MapMode.WriteDiscard, MapFlags.None);
            VideoFrameRef vidRef = videoStream.ReadFrame();
            byte[] bits=new byte[width*height*2];
            List<byte> drawed=new List<byte>();
            Marshal.Copy(vidRef.Data,bits,0,width*height*2);
            mapSubresource.Data.Seek(0, SeekOrigin.Begin);
            for (int i = 0; i < width*height; i++)
            {
                UInt16 col=BitConverter.ToUInt16(bits,i*2);
                byte color = (byte) Math.Min(col*255/MaxDistance,255);
                drawed.Add(color);
                drawed.Add(color);
                drawed.Add(color);
                drawed.Add(255);
            }
            mapSubresource.Data.WriteRange(drawed.ToArray());
            context.DeviceManager.Context.UnmapSubresource(TextureResource,0);
        }

        public bool NeedUpdate { get; private set; }

        public void Dispose()
        {
            TextureResource.Dispose();
        }
    }
}
