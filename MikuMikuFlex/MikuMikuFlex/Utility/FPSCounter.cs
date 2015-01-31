using System.Collections.Generic;
using System.Timers;

namespace MMF.Utility
{
    /// <summary>
    ///     FPSカウンタ
    /// </summary>
    public class FPSCounter
    {
        /// <summary>
        ///     コンストラクタ
        /// </summary>
        public FPSCounter()
        {
            frameHistory = new Queue<int>();
            AvarageSpan = 10;
            FpsTimer = new Timer(1000d);
            FpsTimer.Elapsed += Tick;
        }

        /// <summary>
        ///     FPSの履歴
        /// </summary>
        private Queue<int> frameHistory { get; set; }

        /// <summary>
        ///     FPSカウンタ
        /// </summary>
        private int counter { get; set; }

        /// <summary>
        ///     FPSをカウントするタイマー
        /// </summary>
        public Timer FpsTimer { get; private set; }

        /// <summary>
        ///     FPSの平均をとる秒数
        /// </summary>
        public int AvarageSpan { get; set; }

        private bool isCached;

        private float cachedFPS;

        /// <summary>
        ///     FPS
        /// </summary>
        public float FPS
        {
            get
            {
                if (!isCached)
                {
                    int sum = 0;
                    foreach (int i in frameHistory)
                    {
                        sum += i;
                    }
                    cachedFPS=sum/(float) frameHistory.Count;
                    isCached = true;
                    return cachedFPS;
                }
                return cachedFPS;
            }
        }

        /// <summary>
        ///     FPSカウントをスタートします
        /// </summary>
        public void Start()
        {
            counter = 0;
            FpsTimer.Start();
        }

        /// <summary>
        ///     フレームを進める
        /// </summary>
        public void CountFrame()
        {
            counter++;
        }

        private void Tick(object sender, ElapsedEventArgs args)
        {
            if (frameHistory.Count > AvarageSpan) frameHistory.Dequeue();
            frameHistory.Enqueue(counter);
            counter = 0;
            isCached = false;
        }
    }
}