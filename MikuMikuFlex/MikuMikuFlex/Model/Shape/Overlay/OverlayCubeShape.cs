using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SlimDX;

namespace MMF.Model.Shape.Overlay
{
    public class OverlayCubeShape:CubeShape
    {
        private Vector4 baseColor;
        private Vector4 overlayColor;
        public OverlayCubeShape(RenderContext context, Vector4 color,Vector4 overlayColor) : base(context, color)
        {
            this.baseColor = color;
            this.overlayColor = overlayColor;
        }

        public override void HitTestResult(bool result, bool mouseState, Point mousePosition)
        {
            base.HitTestResult(result, mouseState, mousePosition);
            _color = result ? overlayColor : baseColor;
        }
    }
}
