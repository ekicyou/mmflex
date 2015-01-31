using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using MMDFileParser.MotionParser;
using MMF.Bone;
using MMF.Model;
using MMF.Morph;

namespace MMF.Motion
{
    /// <summary>
    /// モーションを管理するクラス
    /// </summary>
    public class MMDMotion : IMotionProvider
    {
        /// <summary>
        /// ボーン
        /// </summary>
        private PMXBone[] bones;

        /// <summary>
        /// ボーンモーションのリスト
        /// </summary>
        private readonly List<BoneMotion> boneMotions = new List<BoneMotion>();

        /// <summary>
        /// モーフモーションのリスト
        /// </summary>
        private readonly List<MorphMotion> morphMotions = new List<MorphMotion>(); 

        /// <summary>
        ///     モーションデータ
        /// </summary>
        private MotionData motionData;

        /// <summary>
        /// モーション再生終了後の挙動
        /// </summary>
        private ActionAfterMotion actionAfterMotion = ActionAfterMotion.Nothing;

        /// <summary>
        /// このモーションが再生中か否か
        /// </summary>
        private bool isPlaying;

        /// <summary>
        /// アタッチ済みか否か
        /// </summary>
        private bool isAttached;

        /// <summary>
        /// 親を無視するか否か
        /// </summary>
        private bool ignoreParent;

        /// <summary>
        /// コンストラクタの中身
        /// </summary>
        private void _MMDMotion(Stream fs, bool ignoreParent)
        {
            this.ignoreParent = ignoreParent;
            motionData = MotionData.getMotion(fs);
        }

        /// コンストラクタ
        /// </summary>
        /// <param name="filePath">VMEファイル名</param>
        /// <param name="ignoreParent">"全ての親"を無視するか否か</param>        
        public MMDMotion(string filePath, bool ignoreParent)
        {
            using (var fs = new FileStream(filePath, FileMode.Open))
            {
                _MMDMotion(fs, ignoreParent);
            }
        }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="fs">VMEファイルのファイルストリーム</param>
        /// <param name="ignoreParent">"全ての親"を無視するか否か</param>
        public MMDMotion(Stream fs, bool ignoreParent)
        {
            _MMDMotion(fs, ignoreParent);
        }

        /// <summary>
        /// IMotionProviderメンバーの実装
        /// </summary>
        public event EventHandler<EventArgs> FrameTicked;
        public event EventHandler<ActionAfterMotion> MotionFinished;
        public float CurrentFrame { get; set; }
        public int FinalFrame { get; private set; }
        public bool IsAttached { get { return isAttached; } }
        public void Stop() { isPlaying = false; }

        /// <summary>
        /// IMotionProviderメンバーの実装
        /// </summary>
        public void AttachMotion(PMXBone[] bones)
        {
            this.bones = bones;

            // データのアタッチ
            AttachBoneFrameDataToBoneMotion();
            AttachMorphFrameDataToMorphMotion();

            //フレームのソートと最終フレームの検出
            foreach (var boneMotion in boneMotions)
            {
                boneMotion.SortBoneFrameDatas();
                FinalFrame = Math.Max((int)boneMotion.GetFinalFrameNumber(), FinalFrame);
            }
            foreach (var morphMotion in morphMotions)
            {
                morphMotion.SortMorphFrameDatas();
            }

            isAttached = true;
        }

        /// <summary>
        /// IMotionProviderメンバーの実装
        /// </summary>
        public void Tick(int fps, float elapsedTime, IMorphManager morphManager)
        {
            // 行列の更新
            foreach (var boneMotion in boneMotions) boneMotion.ReviseBone(CurrentFrame);
            foreach (var morphMotion in morphMotions) morphManager.ApplyMorphProgress(morphMotion.GetMorphValue((ulong)CurrentFrame), morphMotion.MorphName);

            if (!isPlaying) return;
            CurrentFrame += (float)elapsedTime * fps;
            if (CurrentFrame >= FinalFrame) CurrentFrame = FinalFrame;
            if (FrameTicked != null) FrameTicked(this, new EventArgs());
            if (CurrentFrame >= FinalFrame)
            {
                if (MotionFinished != null) MotionFinished(this, actionAfterMotion);
                if (actionAfterMotion == ActionAfterMotion.Replay) CurrentFrame = 1.0e-3f;
            }
        }

        /// <summary>
        /// IMotionProviderメンバーの実装
        /// </summary>
        public void Start(float frame, ActionAfterMotion action)
        {
            if (frame > FinalFrame) throw new InvalidOperationException("最終フレームを超えた場所から再生を求められました。");
            CurrentFrame = frame;
            isPlaying = true;
            actionAfterMotion = action;
        }

        /// <summary>
        /// ボーンモーションにボーンフレームデータをアタッチする
        /// </summary>
        private void AttachBoneFrameDataToBoneMotion()
        {
            foreach (var boneFrameData in motionData.boneFrameList.boneFrameDatas)
            {
                if (ignoreParent && boneFrameData.BoneName.Equals("全ての親")) continue;
                if (!bones.Any(b => b.BoneName.Equals(boneFrameData.BoneName))) continue;
                var bone = bones.Single(b => b.BoneName.Equals(boneFrameData.BoneName));
                if (!boneMotions.Any(bm => bm.GetBoneName().Equals(boneFrameData.BoneName)))
                {
                    var boneMotion = new BoneMotion(bone);
                    boneMotion.AddBoneFrameData(boneFrameData);
                    boneMotions.Add(boneMotion);
                    continue;
                }
                boneMotions.Single(bm => bm.GetBoneName().Equals(boneFrameData.BoneName)).AddBoneFrameData(boneFrameData);
            }
        }

        /// <summary>
        /// モーフモーションにモーフフレームデータをアタッチする
        /// </summary>
        private void AttachMorphFrameDataToMorphMotion()
        {
            foreach (var morphFrameData in motionData.morphFrameList.morphFrameDatas)
            {
                if (!morphMotions.Any(mm => mm.MorphName.Equals(morphFrameData.Name)))
                {
                    var morphMotion = new MorphMotion(morphFrameData.Name);
                    morphMotion.AddMorphFrameData(morphFrameData);
                    morphMotions.Add(morphMotion);
                }
                morphMotions.Single(mm => mm.MorphName.Equals(morphFrameData.Name)).AddMorphFrameData(morphFrameData);
            }
        }


    }
}