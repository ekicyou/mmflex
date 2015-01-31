using System.Diagnostics;

namespace MMF.Motion
{
    /// <summary>
    ///     モーションに関するタイマー
    /// </summary>
    public class MotionTimer
    {
        private readonly RenderContext _context;

         
        /// <summary>
        /// MMDの1秒辺りのフレーム数
        /// 通常30。
        /// </summary>
        public int MotionFramePerSecond = 30;

        /// <summary>
        /// タイマーの更新速度
        /// 理想とするfps
        /// 一般的には60
        /// </summary>
        public int TimerPerSecond = 300;

        public float ElapesedTime { get; private set; }

        public static Stopwatch stopWatch;

        private long lastMillisecound = 0;

        static MotionTimer()
        {
            stopWatch=new Stopwatch();
            stopWatch.Start();
        }


        /// <summary>
        ///     コンストラクタ
        /// </summary>
        /// <param name="updateGrobal">アップデート時に呼ばれるハンドら</param>
        public MotionTimer(RenderContext context)
        {
            _context = context;
        }

        public void TickUpdater()
        {
            if (lastMillisecound == 0)
            {
                lastMillisecound = stopWatch.ElapsedMilliseconds;
            }
            else
            {
                long currentMillisecound = stopWatch.ElapsedMilliseconds;
                if (currentMillisecound - lastMillisecound > 1000/TimerPerSecond)
                {
                    ElapesedTime = currentMillisecound - lastMillisecound;
                    _context.UpdateWorlds();
                    lastMillisecound = stopWatch.ElapsedMilliseconds;
                }
            }
        }

    }
}