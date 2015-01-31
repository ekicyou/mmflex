using System;
using System.Diagnostics;
using SlimDX;
using SlimDX.Direct2D;

namespace MMF.Sprite.D2D
{
    public class D2DSpriteSolidColorBrush : IDisposable
    {
        private readonly D2DSpriteBatch batch;
        private Color4 color;

        internal D2DSpriteSolidColorBrush(D2DSpriteBatch batch, Color4 color)
        {
            this.batch = batch;
            this.color = color;
            batch.BatchDisposing += batch_BatchDisposing;
            Brush = new SolidColorBrush(batch.DWRenderTarget, color);
        }

        public Color4 Color
        {
            get { return color; }
            set
            {
                color = value;
                if (Brush != null) Brush.Color = color;
            }
        }

        public SolidColorBrush Brush { get; private set; }

        public void Dispose()
        {
            if (Brush != null && !Brush.Disposed) Brush.Dispose();
            batch.BatchDisposing -= batch_BatchDisposing;
            GC.SuppressFinalize(this);
        }

        void batch_BatchDisposing(object sender, EventArgs e)
        {
            Dispose();
        }

        ~D2DSpriteSolidColorBrush()
        {
            if (Brush != null && !Brush.Disposed) Brush.Dispose();
            Debug.WriteLine("D2DSpriteSolidColorBrushはDisposableですがDisposeされませんでした。");
        }

        public static implicit operator SolidColorBrush(D2DSpriteSolidColorBrush brush)
        {
            return brush.Brush;
        }
    }
}