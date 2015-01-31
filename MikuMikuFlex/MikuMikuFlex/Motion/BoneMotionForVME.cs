using System;
using System.Collections.Generic;
using System.Linq;
using MMF.Bone;
using MMF.Utility;
using OpenMMDFormat;

namespace MMF.Motion
{
    /// <summary>
    /// VMEモーションデータを使ってボーンを更新するクラス
    /// </summary>
    internal class BoneMotionForVME
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
        /// <param name="boneFrames">ボーンのモーションデータ</param>
        public BoneMotionForVME(PMXBone bone, List<BoneFrame> boneFrames)
        {
            this.bone = bone;
            foreach (var boneFrame in boneFrames) frameManager.AddFrameData(boneFrame);
            if (!frameManager.IsSorted()) throw new Exception("VMEデータがソートされていません");
        }

        /// <summary>
        /// 最後のフレーム番号を取得する
        /// </summary>
        /// <returns>最後のフレーム番号</returns>
        public uint GetFinalFrame()
        {
            return frameManager.GetFinalFrameNumber();
        }

        /// <summary>
        /// ボーンを指定したフレーム番号の姿勢に更新する
        /// </summary>
        /// <param name="frameNumber">フレーム番号</param>
        public void ReviseBone(ulong frameNumber)
        {
            // 現在のフレームの前後のキーフレームを探す
            MMDFileParser.IFrameData pastFrame, futureFrame;
            frameManager.SearchKeyFrame(frameNumber, out pastFrame, out futureFrame);
            var pastBoneFrame = (BoneFrame)pastFrame;
            var futureBoneFrame = (BoneFrame)futureFrame;

            // 現在のフレームの前後キーフレーム間での進行度を求めてペジェ関数で変換する
            float s = (pastBoneFrame.frameNumber == futureBoneFrame.frameNumber)? 0 :
                (float)(frameNumber - pastBoneFrame.frameNumber) / (float)(futureBoneFrame.frameNumber - pastBoneFrame.frameNumber); // 進行度
            BezInterpolParams p = pastBoneFrame.interpolParameters;
            float s_X, s_Y, s_Z,s_R;
            if (p != null)
            {
                s_X = BezEvaluate(p.X1, p.X2, s);
                s_Y = BezEvaluate(p.Y1, p.Y2, s);
                s_Z = BezEvaluate(p.Z1, p.Z2, s);
                s_R = BezEvaluate(p.R1, p.R2, s); // ペジェ変換後の進行度
            }
            else
            {//ベジェ曲線のパラメータがないときは線形補完の量としてsを利用する
                s_X = s_Y = s_Z = s_R = s;
            }
            // ボーンを更新する
            bone.Translation = new SlimDX.Vector3(
                CGHelper.Lerp(pastBoneFrame.position.x, futureBoneFrame.position.x, s_X),
                CGHelper.Lerp(pastBoneFrame.position.y, futureBoneFrame.position.y, s_Y),
                CGHelper.Lerp(pastBoneFrame.position.z, futureBoneFrame.position.z, s_Z));
            bone.Rotation = SlimDX.Quaternion.Slerp(pastBoneFrame.rotation.ToSlimDX(), futureBoneFrame.rotation.ToSlimDX(), s_R);
        }

        /// <summary>
        /// ベジェ関数
        /// </summary>
        /// <param name="v1">ベジェ形状パラメータ1</param>
        /// <param name="v2">ベジェ形状パラメータ2</param>
        /// <param name="s">変数</param>
        /// <returns>ベジェ関数値</returns>
        private float BezEvaluate(bvec2 v1, bvec2 v2, float s)
        {
            var curve = new MMDFileParser.MotionParser.BezierCurve();
            curve.v1 = v1.ToSlimDX() / 127;
            curve.v2 = v2.ToSlimDX() / 127;
            return curve.Evaluate(s);
        }

    }
}
