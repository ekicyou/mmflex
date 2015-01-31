using System;
using System.Diagnostics;
using SlimDX.DirectWrite;

namespace MMF.Sprite.D2D
{
    public class D2DSpriteTextformat : IDisposable
    {
        internal D2DSpriteTextformat(D2DSpriteBatch batch, string fontFamiry, int size, FontWeight weight,
            FontStyle style, FontStretch stretch, string locale)
        {
            batch.BatchDisposing += batch_BatchDisposing;
            Format = new TextFormat(batch.context.DWFactory, fontFamiry, weight, style, stretch, size, locale);
        }

        void batch_BatchDisposing(object sender, EventArgs e)
        {
            Dispose();
        }

        /// <summary>
        ///     フォーマット
        /// </summary>
        public TextFormat Format { get; private set; }

        public void Dispose()
        {
            if (Format != null && !Format.Disposed) Format.Dispose();
            GC.SuppressFinalize(this);
        }

        ~D2DSpriteTextformat()
        {
            Debug.WriteLine("D2DSpriteTextformatはDisposableですがDisposeされませんでした。");
            if (Format != null && !Format.Disposed) Format.Dispose();
        }

        public static implicit operator TextFormat(D2DSpriteTextformat format)
        {
            return format.Format;
        }
    }
}