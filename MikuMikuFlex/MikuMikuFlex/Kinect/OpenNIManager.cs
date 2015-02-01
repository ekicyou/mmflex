using System;
using System.Collections.Generic;
using NiTEWrapper;
using OpenNIWrapper;

namespace MMF.Kinect
{
    public class OpenNIManager:IDisposable
    {
        private static OpenNIManager Instance;

        private static List<Device> createdDevices=new List<Device>(); 

        public static DeviceInfo[] ConnectedDevices;

        public static void ShutDown()
        {
            if(Instance!=null)Instance.Dispose();
        }

        internal OpenNIManager()
        {
        }

        ~OpenNIManager()
        {
            Dispose();
        }

        public void Dispose()
        {
            foreach (var createdDevice in createdDevices)
            {
                createdDevice.Close();
            }
            OpenNI.Shutdown();
            NiTE.Shutdown();
            GC.SuppressFinalize(this);
        }

        public static void Initialize()
        {
            OpenNI.Shutdown();//例外などで前回シャットダウンされなかった場合用
            OpenNI.Initialize();
            NiTE.Initialize();
            ConnectedDevices=OpenNI.EnumerateDevices();
            OpenNI.OnDeviceConnected += OpenNI_onDeviceConnected;
            OpenNI.OnDeviceDisconnected+=OpenNI_onDeviceConnected;
            OpenNI.OnDeviceStateChanged += OpenNI_onDeviceStateChanged;
            Instance=new OpenNIManager();
        }

        static void OpenNI_onDeviceStateChanged(DeviceInfo Device, OpenNI.DeviceState state)
        {
            ConnectedDevices = OpenNI.EnumerateDevices();
        }

        static void OpenNI_onDeviceConnected(DeviceInfo Device)
        {
            ConnectedDevices = OpenNI.EnumerateDevices();
        }

        public static KinectDeviceManager getDevice(string uri=null)
        {
            Device d = Device.Open(uri);
            createdDevices.Add(d);
            return new KinectDeviceManager(d);
        }
    }
}
