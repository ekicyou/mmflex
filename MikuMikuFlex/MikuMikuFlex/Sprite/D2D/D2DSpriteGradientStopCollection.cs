using System;
using System.Diagnostics;
using SlimDX.Direct2D;

namespace MMF.Sprite.D2D
{
    public class D2DSpriteGradientStopCollection:IDisposable
    {
        public GradientStopCollection GradientStopCollection;
        private D2DSpriteBatch _batch;
        private GradientStop[] _stops;
        private Gamma _gamma;
        private ExtendMode _extendMode;

        public D2DSpriteGradientStopCollection(D2DSpriteBatch batch,GradientStop[] stops,Gamma gamma,ExtendMode mode)
        {
            batch.BatchDisposing += batch_BatchDisposing;
            _extendMode = mode;
            _gamma = gamma;
            _stops = stops;
            _batch = batch;
        }

        void batch_BatchDisposing(object sender, EventArgs e)
        {
            Dispose();
        }

        ~D2DSpriteGradientStopCollection()
        {
            Debug.WriteLine("D2DSpriteGradientStopCollectionはIDisposableですが、Disposeされませんでした。");
            Dispose();
        }


        public void Dispose()
        {
            if (GradientStopCollection != null && !GradientStopCollection.Disposed) GradientStopCollection.Dispose();
            GC.SuppressFinalize(this);
        }
    }
}
