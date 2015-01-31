using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMF.Model;

namespace MMF.DeviceManager
{
    public interface IHitTestable:IDrawable
    {
        void RenderHitTestBuffer(float col);

        void HitTestResult(bool result,bool mouseState,Point mousePosition);
    }
}
