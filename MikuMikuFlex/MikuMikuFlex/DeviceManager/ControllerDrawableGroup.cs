using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SlimDX.Direct3D11;

namespace MMF.DeviceManager
{
    public class ControllerDrawableGroup:DrawableGroup
    {
        public ControllerDrawableGroup(int priorty, string groupName, RenderContext context) : base(priorty, groupName, context)
        {
        }

        protected override void PreDraw()
        {
            base.PreDraw();
            _context.DeviceManager.Context.ClearDepthStencilView(_context.CurrentRenderDepthStencilTarget,DepthStencilClearFlags.Depth, 1,0);
        }
    }
}
