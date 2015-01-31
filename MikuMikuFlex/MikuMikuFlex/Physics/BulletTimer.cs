using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMF.Physics
{
    /// <summary>
    /// 経過時間[ms]を計るクラス
    /// </summary>
    internal class BulletTimer
    {
        /// <summary>
        /// 時計
        /// </summary>
        private System.Diagnostics.Stopwatch stopWatch = new System.Diagnostics.Stopwatch();

        /// <summary>
        /// 前回時計を見た時の時刻
        /// </summary>
        private long lastTime = 0;

        /// <summary>
        /// コンストラクタ
        /// </summary>
        public BulletTimer()
        {
            stopWatch.Start();
        }

        /// <summary>
        /// 前回この関数を呼んでからの経過時間[ms]を得る
        /// </summary>
        /// <returns>経過時間[ms]</returns>
        public long GetElapsedTime()
        {
            var currentTime = stopWatch.ElapsedMilliseconds;
            var elapsedTime = currentTime - lastTime;
            lastTime = currentTime;
            return elapsedTime;
        }
    }
}
