using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using MMF.Sprite;
using OpenNIWrapper;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;
using Device = OpenNIWrapper.Device;
using MapFlags = SlimDX.Direct3D11.MapFlags;

namespace MMF.Kinect
{
    public class IRTexture:IDynamicTexture
    {
       private RenderContext context;
        public Texture2D TextureResource { get; set; }
        private VideoStream videoStream;

        public IRTexture(RenderContext context,KinectDeviceManager device)
        {
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
            videoStream = device.KinnectDevice.CreateVideoStream(Device.SensorType.IR);
            videoStream.Start();
            TextureResourceView = new ShaderResourceView(context.DeviceManager.Device, TextureResource);
            NeedUpdate = true;
        }

        public ShaderResourceView TextureResourceView { get; private set; }

        public void UpdateTexture()
        {
            int width = 640;
            int height = 480;
            DataBox mapSubresource = context.DeviceManager.Context.MapSubresource(TextureResource,0, MapMode.WriteDiscard, MapFlags.None);
            VideoFrameRef vidRef = videoStream.readFrame();
            byte[] bits=new byte[width*height];
            List<byte> drawed=new List<byte>();
            Marshal.Copy(vidRef.Data,bits,0,width*height);
            mapSubresource.Data.Seek(0, SeekOrigin.Begin);
            for (int i = 0; i < width*height; i++)
            {
                drawed.Add(bits[i]);
                drawed.Add(bits[i]);
                drawed.Add(bits[i]);
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
