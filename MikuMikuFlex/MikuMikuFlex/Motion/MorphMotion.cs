using System.Collections.Generic;
using MMDFileParser.MotionParser;
using MMF.Utility;
using System.Linq;

namespace MMF.Motion
{
    public class MorphMotion
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
        public MorphMotion(string morphName)
        {
            MorphName = morphName;
        }

        /// <summary>
        /// モーフフレームデータを付け加える
        /// </summary>
        /// <param name="morphFrameData">モーフフレームデータ</param>
        public void AddMorphFrameData(MorphFrameData morphFrameData)
        {
            frameManager.AddFrameData(morphFrameData);
        }

        /// <summary>
        /// モーフフレームデータリストをソートする
        /// </summary>
        public void SortMorphFrameDatas()
        {
            frameManager.SortFrameDatas();
        }

        /// <summary>
        /// 指定したフレーム番号のモーフの値を取得する
        /// </summary>
        /// <param name="frame">フレーム番号</param>
        public float GetMorphValue(float frameNumber)
        {
            // 現在のフレームの前後のキーフレームを探す
            MMDFileParser.IFrameData pastFrame, futureFrame;
            frameManager.SearchKeyFrame(frameNumber, out pastFrame, out futureFrame);
            var pastMorphFrame = (MorphFrameData)pastFrame;
            var futureMorphFrame = (MorphFrameData)futureFrame;

            // 現在のフレームの前後キーフレーム間での進行度を求めてペジェ関数で変換する
            float s = (futureMorphFrame.FrameNumber == pastMorphFrame.FrameNumber) ? 0 :
                (float)(frameNumber - pastMorphFrame.FrameNumber) / (float)(futureMorphFrame.FrameNumber - pastMorphFrame.FrameNumber); // 進行度
            return CGHelper.Lerp(pastMorphFrame.MorphValue, futureMorphFrame.MorphValue, s);
        }

    }
}
