using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser
{
    public class FrameManager
    {
        private List<IFrameData> frameDatas = new List<IFrameData>();

        /// <summary>
        /// 前回のフレームの過去キーフレームのインデックス
        /// </summary>
        private int beforePastFrameIndex = 0;

        /// <summary>
        /// フレームデータを追加
        /// </summary>
        /// <param name="frameData">フレームデータ</param>
        public void AddFrameData(IFrameData frameData)
        {
            frameDatas.Add(frameData);
        }

        /// <summary>
        /// フレームデータをソート
        /// </summary>
        public void SortFrameDatas()
        {
            frameDatas.Sort();
        }

        /// <summary>
        /// フレームの昇順に並んでいることをチェックする
        /// </summary>
        public bool IsSorted()
        {
            var prev = frameDatas[0].FrameNumber;
            foreach (var frameData in frameDatas)
            {
                if (prev > frameData.FrameNumber) return false;
                prev = frameData.FrameNumber;
            }
            return true;
        }

        /// <summary>
        /// フレームデータの最後のフレーム番号を取得
        /// </summary>
        /// <returns>フレームデータの最後のフレーム番号</returns>
        public uint GetFinalFrameNumber()
        {
            return frameDatas.Last().FrameNumber;
        }

        /// <summary>
        /// 現在のフレームの前後のキーフレームを探す
        /// </summary>
        /// <param name="frameNumber">現在のフレーム番号</param>
        /// <param name="pastFrame">過去のキーフレーム</param>
        /// <param name="futureFrame">未来のキーフレーム</param>
        public void SearchKeyFrame(float frameNumber, out MMDFileParser.IFrameData pastFrame, out MMDFileParser.IFrameData futureFrame)
        {
            // 現在のフレームが最初のキーフレームより前にある場合
            if (frameNumber < frameDatas.First().FrameNumber)
            {
                pastFrame = futureFrame = frameDatas.First();
                return;
            }

            // 現在のフレームが最後のキーフレームより後にある場合
            if (frameNumber >= frameDatas.Last().FrameNumber)
            {
                pastFrame = futureFrame = frameDatas.Last();
                return;
            }

            // 現在のフレームの後のキーフレームのインデックスを探す
            // 高速化のため、現在のフレームが前回の過去キーフレームより進んでいれば、前回の過去キーフレームから探す
            int futureFrameIndex;
            if (frameDatas[beforePastFrameIndex].FrameNumber < frameNumber)
                futureFrameIndex = frameDatas.FindIndex(beforePastFrameIndex, b => b.FrameNumber > frameNumber);
            else
                futureFrameIndex = frameDatas.FindIndex(b => b.FrameNumber > frameNumber);

            // 現在のフレームの前後のキーフレームを出力
            pastFrame = frameDatas[futureFrameIndex - 1];
            futureFrame = frameDatas[futureFrameIndex];

            // 現在の過去キーフレームを記憶しておく
            beforePastFrameIndex = futureFrameIndex - 1;
        }
    }
}
