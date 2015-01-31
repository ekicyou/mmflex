using System;
using System.Diagnostics;
using SlimDX.Direct2D;

namespace MMF.Sprite.D2D
{
    public class D2DSpriteBitmapBrush:IDisposable
    {
        public BitmapBrush Brush;
        private BitmapBrushProperties _bitmapBrushProperties;
        private D2DSpriteBatch _batch;
        private D2DSpriteBitmap _bitmap;

        public D2DSpriteBitmapBrush(D2DSpriteBatch batch,D2DSpriteBitmap bitmap,BitmapBrushProperties bbp)
        {
            _bitmap = bitmap;
            _batch = batch;
            _bitmapBrushProperties = bbp;
            Brush=new BitmapBrush(batch.DWRenderTarget,_bitmap,bbp);
            _batch.BatchDisposing += _batch_BatchDisposing;
        }

        void _batch_BatchDisposing(object sender, EventArgs e)
        {
            Dispose();
        }

        ~D2DSpriteBitmapBrush()
        {
            Debug.WriteLine("D2DSpriteBitmapBrushはIDisposableですが、Disposeされませんでした。");
            if (Brush != null && !Brush.Disposed) Brush.Dispose();
        }

        public void Dispose()
        {
            if(Brush!=null&&!Brush.Disposed)Brush.Dispose();
            GC.SuppressFinalize(this);
        }

        public static implicit operator BitmapBrush(D2DSpriteBitmapBrush brush)
        {
            return brush.Brush;
        }
    }
}
