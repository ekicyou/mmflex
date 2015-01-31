using System;
using System.Collections.Generic;
using System.Linq;
using MMF.Utility;
using OpenMMDFormat;

namespace MMF.Motion
{
    /// <summary>
    /// ひとつのモーフのモーションの集合(VME版)
    /// フレームの昇順に並んでいる
    /// </summary>
    class MorphMotionForVME
    {
        /// <summary>
        /// フレームマネージャ
        /// </summary>
        private MMDFileParser.FrameManager frameManager = new MMDFileParser.FrameManager();

        /// <summary>
        /// モーフの名前
        /// </summary>
        public string MorphName { get; private set; }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="morphName">モーフの名前</param>
        /// <param name="morphFrames">モーフのモーションデータ</param>
        public MorphMotionForVME(string morphName, List<MorphFrame> morphFrames)
        {
            this.MorphName = MorphName;
            foreach (var morphFrame in morphFrames) frameManager.AddFrameData(morphFrame);
            if (!frameManager.IsSorted()) throw new Exception("VMEデータがソートされていません");
        }

        /// <summary>
        /// 指定したフレーム番号のモーフの値を取得する
        /// </summary>
        /// <param name="frameNumber">フレーム番号</param>
        public float GetMorphValue(ulong frameNumber)
        {
            // 現在のフレームの前後のキーフレームを探す
            MMDFileParser.IFrameData pastFrame, futureFrame;
            frameManager.SearchKeyFrame(frameNumber, out pastFrame, out futureFrame);
            var pastMorphFrame = (MorphFrame)pastFrame;
            var futureMorphFrame = (MorphFrame)futureFrame;

            // 現在のフレームの前後キーフレーム間での進行度を求める
            float s = (futureMorphFrame.frameNumber == pastMorphFrame.frameNumber)? 0 : 
                (float)(frameNumber - pastMorphFrame.frameNumber) / (float)(futureMorphFrame.frameNumber - pastMorphFrame.frameNumber); // 進行度

            // 線形補完で値を求める
            return CGHelper.Lerp(pastMorphFrame.value, futureMorphFrame.value, s);
        }

    }
}
