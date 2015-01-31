using System;
using System.Collections.Generic;
using System.Linq;
using MMDFileParser.MotionParser;
using MMF.Bone;
using MMF.Utility;
using SlimDX;

namespace MMF.Motion
{
    /// <summary>
    /// VMDモーションデータを使ってボーンを更新するクラス
    /// </summary>
    internal class BoneMotion
    {
        /// <summary>
        /// ボーン
        /// </summary>
        private PMXBone bone;

        /// <summary>
        /// フレームマネージャ
        /// </summary>
        private MMDFileParser.FrameManager frameManager = new MMDFileParser.FrameManager();

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="bone">ボーン</param>
        public BoneMotion(PMXBone bone)
        {
            this.bone = bone;
        }

        /// <summary>
        /// ボーンフレームデータを付け加える
        /// </summary>
        /// <param name="boneFrameData">ボーンフレームデータ</param>
        public void AddBoneFrameData(BoneFrameData boneFrameData)
        {
            frameManager.AddFrameData(boneFrameData);
        }

        /// <summary>
        /// ボーンフレームデータリストをソートする
        /// </summary>
        public void SortBoneFrameDatas()
        {
            frameManager.SortFrameDatas();
        }

        /// <summary>
        /// ボーンフレームデータの最後のフレーム番号を取得する
        /// </summary>
        /// <returns>ボーンフレームデータの最後のフレーム番号</returns>
        public uint GetFinalFrameNumber()
        {
            return frameManager.GetFinalFrameNumber();
        }

        /// <summary>
        /// ボーン名を取得
        /// </summary>
        /// <returns>ボーン名</returns>
        public String GetBoneName() { return bone.BoneName; }

        /// <summary>
        /// ボーンを指定したフレーム番号の姿勢に更新する
        /// </summary>
        /// <param name="frameNumber">現在のフレーム番号</param>
        public void ReviseBone(float frameNumber)
        {
            // 現在のフレームの前後のキーフレームを探す
            MMDFileParser.IFrameData pastFrame, futureFrame;
            frameManager.SearchKeyFrame(frameNumber, out pastFrame, out futureFrame);
            var pastBoneFrame = (BoneFrameData)pastFrame;
            var futureBoneFrame = (BoneFrameData)futureFrame;

            // 現在のフレームの前後キーフレーム間での進行度を求めてペジェ関数で変換する
            float s = (futureBoneFrame.FrameNumber == pastBoneFrame.FrameNumber) ? 0 : 
                (float)(frameNumber - pastBoneFrame.FrameNumber) / (float)(futureBoneFrame.FrameNumber - pastBoneFrame.FrameNumber); // 進行度
            var ss = new float[4];
            for (int i = 0; i < ss.Length; ++i) ss[i] = pastBoneFrame.Curves[i].Evaluate(s);

            // ボーンを更新する
            bone.Translation = CGHelper.ComplementTranslate(pastBoneFrame, futureBoneFrame, new Vector3(ss[0], ss[1], ss[2]));
            bone.Rotation = CGHelper.ComplementRotateQuaternion(pastBoneFrame, futureBoneFrame, ss[3]);
        }

    }
}