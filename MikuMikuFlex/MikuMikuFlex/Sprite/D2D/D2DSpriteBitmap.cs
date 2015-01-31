using System;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using SlimDX;
using SlimDX.Direct2D;
using SlimDX.DXGI;
using Bitmap = System.Drawing.Bitmap;
using PixelFormat = SlimDX.Direct2D.PixelFormat;

namespace MMF.Sprite.D2D
{
    /// <summary>
    /// ビットマップ読み込み補助クラス
    /// </summary>
    public class D2DSpriteBitmap:IDisposable
    {
        ~D2DSpriteBitmap()
        {
            Debug.WriteLine("D2DSpriteBitmapはIDisposableですが、Disposeされませんでした。");
            if (SpriteBitmap != null && !SpriteBitmap.Disposed) SpriteBitmap.Dispose();
        }

        private Bitmap orgBitmap;
        private D2DSpriteBatch batch;
        public SlimDX.Direct2D.Bitmap SpriteBitmap { get; private set; }



        internal D2DSpriteBitmap(D2DSpriteBatch batch,Stream fs)
        {
            this.batch = batch;
            batch.BatchDisposing += batch_BatchDisposing;
            orgBitmap = (Bitmap)Bitmap.FromStream(fs);
            CreateBitmap();
        }

        void batch_BatchDisposing(object sender, EventArgs e)
        {
            Dispose();
        }


        private void CreateBitmap()
        {
            BitmapData bitmapData = orgBitmap.LockBits(new Rectangle(0, 0, orgBitmap.Width, orgBitmap.Height),
                ImageLockMode.ReadOnly, System.Drawing.Imaging.PixelFormat.Format32bppRgb);
            using (DataStream dataStream = new DataStream(bitmapData.Scan0, bitmapData.Stride*bitmapData.Height, true, false))
            {
                PixelFormat format = new PixelFormat(Format.B8G8R8A8_UNorm, AlphaMode.Premultiplied);
                BitmapProperties properties = new BitmapProperties();
                properties.HorizontalDpi = properties.VerticalDpi = 96;
                properties.PixelFormat = format;
                if (SpriteBitmap != null && !SpriteBitmap.Disposed) SpriteBitmap.Dispose();
                SpriteBitmap = new SlimDX.Direct2D.Bitmap(batch.DWRenderTarget, new Size(orgBitmap.Width, orgBitmap.Height),
                    dataStream, bitmapData.Stride, properties);
                orgBitmap.UnlockBits(bitmapData);
            }
        }

        public void Dispose()
        {
            if (SpriteBitmap != null && !SpriteBitmap.Disposed) SpriteBitmap.Dispose();
            GC.SuppressFinalize(this);
        }

        public static implicit operator SlimDX.Direct2D.Bitmap(D2DSpriteBitmap bitmap)
        {
            return bitmap.SpriteBitmap;
        }
    }
}