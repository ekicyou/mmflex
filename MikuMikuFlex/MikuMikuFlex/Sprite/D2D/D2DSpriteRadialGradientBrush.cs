using System;
using System.Diagnostics;
using SlimDX.Direct2D;

namespace MMF.Sprite.D2D
{
    public class D2DSpriteRadialGradientBrush:IDisposable
    {
        public RadialGradientBrush Brush;

        public D2DSpriteRadialGradientBrush(D2DSpriteBatch batch,GradientStopCollection collection,RadialGradientBrushProperties rgbp)
        {
            Brush=new RadialGradientBrush(batch.DWRenderTarget,collection,rgbp);
        }

        ~D2DSpriteRadialGradientBrush()
        {
            Debug.WriteLine("D2DSpriteRadialGradientBrushはIDisposableですが、Disposeされませんでした。");
            Dispose();
        }

        public void Dispose()
        {
            if (Brush != null && !Brush.Disposed) Brush.Dispose();
            GC.SuppressFinalize(this);
        }
    }
}
