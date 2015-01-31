using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SlimDX.Direct3D11;

namespace MMF.Utility
{
    /// <summary>
    /// PrimitiveTopology=TriangleListの場合用
    /// </summary>
    public class IndexBufferBuilder
    {
        private readonly RenderContext _context;
        private List<uint> list=new List<uint>();

        public IndexBufferBuilder(RenderContext context)
        {
            _context = context;
        }

        public void AddTriangle(uint p, uint q, uint r)
        {
            list.Add(p);
            list.Add(q);
            list.Add(r);
        }

        public void AddSquare(uint p, uint q, uint r,uint s)
        {
            list.Add(p);
            list.Add(q);
            list.Add(s);
            list.Add(s);
            list.Add(q);
            list.Add(r);
        }

        public Buffer build()
        {
            return CGHelper.CreateBuffer(list, _context.DeviceManager.Device, BindFlags.IndexBuffer);
        }
    }
}
