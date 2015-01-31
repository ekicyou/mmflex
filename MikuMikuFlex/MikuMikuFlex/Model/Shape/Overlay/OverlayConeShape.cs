using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SlimDX;

namespace MMF.Model.Shape.Overlay
{
    class OverlayConeShape:ConeShape
    {
        private readonly Vector4 _color;
        private readonly Vector4 _overlayColor;

        public OverlayConeShape(RenderContext context, Vector4 color,Vector4 overlayColor) : base(context, color)
        {
            _color = color;
            _overlayColor = overlayColor;
        }

        public override void HitTestResult(bool result, bool mouseState, Point mousePosition)
        {
            base.HitTestResult(result, mouseState, mousePosition);
            base._color = result ? _overlayColor : _color;
        }
    }
}
