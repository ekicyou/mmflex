using System.Collections.Generic;
using System.IO;
using MMF.Utility;
using SlimDX.Direct3D11;

namespace MMF.Model.PMX
{
    internal class PMXToonTextureManager : IToonTextureManager
    {
        private readonly List<ShaderResourceView> resourceViewsList = new List<ShaderResourceView>();
        private Device _device;

        private ISubresourceLoader _subresourceManager;

        public ShaderResourceView[] ResourceViews
        {
            get { return resourceViewsList.ToArray(); }
        }

        public void Initialize(RenderContext context, ISubresourceLoader subresourceManager)
        {
            _device = context.DeviceManager.Device;
            _subresourceManager = subresourceManager;
            for (int i = 0; i < 11; i++)
            {
                string path = CGHelper.ToonDir + string.Format(@"toon{0}.bmp", i);
                if (File.Exists(path))
                    resourceViewsList.Add(ShaderResourceView.FromFile(context.DeviceManager.Device, path));
                else
                {
                    
                }
            }
        }

        public int LoadToon(string path)
        {
            using (Stream stream = _subresourceManager.getSubresourceByName(path))
            {
                if (stream == null) return 0;
                resourceViewsList.Add(ShaderResourceView.FromStream(_device, stream, (int) stream.Length));
                return resourceViewsList.Count - 1;
            }
        }

        public void Dispose()
        {
            foreach (ShaderResourceView shaderResourceView in resourceViewsList)
            {
                shaderResourceView.Dispose();
            }
        }
    }
}