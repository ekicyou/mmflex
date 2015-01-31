using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using MMF.Bone;
using MMF.Model;
using MMF.Morph;
using OpenMMDFormat;

namespace MMF.Motion
{
    /// <summary>
    /// VME用モーションプロバイダ
    /// </summary>
    public class MMDMotionForVME : IMotionProvider
    {
        /// <summary>
        /// VME用モーション構造体
        /// </summary>
        private VocaloidMotionEvolved vocaloidMotionEvolved;

        /// <summary>
        /// ボーン
        /// </summary>
        private PMXBone[] bones;

        /// <summary>
        /// "全ての親"を無視するか否か
        /// </summary>
        private bool ignoreParent;

        /// <summary>
        /// ボーンのモーションの集合
        /// </summary>
        private readonly List<BoneMotionForVME> boneMotions = new List<BoneMotionForVME>();

        /// <summary>
        /// モーフのモーションの集合
        /// </summary>
        private readonly List<MorphMotionForVME> morphMotions = new List<MorphMotionForVME>();


        /// <summary>
        /// ロード済みか否か
        /// </summary>
        private bool isAttached = false;

        /// <summary>
        /// 再生中か否か
        /// </summary>
        private bool isPlaying = false;

        /// <summary>
        /// モーション再生終了後のアクション
        /// </summary>
        private ActionAfterMotion actionAfterMotion = ActionAfterMotion.Nothing;

        /// <summary>
        /// コンストラクタの中身
        /// </summary>
        private void _MMDMotionFromVME(Stream fs, bool ignoreParent)
        {
            this.ignoreParent = ignoreParent;

            // VME用モーション構造体の取得
            vocaloidMotionEvolved = ProtoBuf.Serializer.Deserialize<VocaloidMotionEvolved>(fs);
        }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="filePath">VMEファイル名</param>
        /// <param name="ignoreParent">"全ての親"を無視するか否か</param>
        public MMDMotionForVME(string filePath, bool ignoreParent)
        {
            using (var fs = new FileStream(filePath, FileMode.Open))
            {
                _MMDMotionFromVME(fs, ignoreParent);
            }
        }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="fs">VMEファイルのファイルストリーム</param>
        /// <param name="ignoreParent">"全ての親"を無視するか否か</param>
        public MMDMotionForVME(Stream fs, bool ignoreParent)
        {
            _MMDMotionFromVME(fs, ignoreParent);
        }

        /// <summary>
        /// IMotionProviderメンバーの実装
        /// </summary>
        public float CurrentFrame { get; set; }
        public int FinalFrame { get; private set; }
        public event EventHandler<EventArgs> FrameTicked;
        public event EventHandler<ActionAfterMotion> MotionFinished;

        /// <summary>
        /// IMotionProviderメンバーの実装
        /// </summary>
        public void AttachMotion(PMXBone[] bones)
        {
            this.bones = bones;

            // ボーンのモーションのセット
            var boneIDDictionary = new Dictionary<ulong, string>();
            foreach (var idTag in vocaloidMotionEvolved.boneIDTable) boneIDDictionary[idTag.id] = idTag.name;
            foreach (var boneFrameTable in vocaloidMotionEvolved.boneFrameTables)
            {
                var boneName = boneIDDictionary[boneFrameTable.id];
                if ((ignoreParent && boneName.Equals("全ての親")) || !bones.Any(b => b.BoneName.Equals(boneName))) continue;
                boneMotions.Add(new BoneMotionForVME(bones.Single(b => b.BoneName.Equals(boneName)), boneFrameTable.frames));
            }

            // モーフのモーションのセット
            var morphIDDictionary = new Dictionary<ulong, string>();
            foreach (var idTag in vocaloidMotionEvolved.morphIDTable) morphIDDictionary[idTag.id] = idTag.name;
            foreach (var morphFrameTable in vocaloidMotionEvolved.morphFrameTables)
            {
                var morphName = morphIDDictionary[morphFrameTable.id];
                morphMotions.Add(new MorphMotionForVME(morphName, morphFrameTable.frames));
            }
            
            // FinalFrameの検出
            foreach (var boneMotion in boneMotions)
            {
                FinalFrame = Math.Max((int)boneMotion.GetFinalFrame(), FinalFrame);
            }

            // ロード完了
            isAttached = true;
        }

        /// <summary>
        /// IMotionProviderメンバーの実装
        /// </summary>
        public void Tick(int fps, float elapsedTime, IMorphManager morphManager)
        {
            // 行列の更新
            foreach (var boneMotion in boneMotions) boneMotion.ReviseBone((ulong)CurrentFrame);
            foreach (var morphMotion in morphMotions) morphManager.ApplyMorphProgress(morphMotion.GetMorphValue((ulong)CurrentFrame), morphMotion.MorphName);

            // 停止中はフレームを進めない
            if (!isPlaying) return;

            // フレームを進める
            CurrentFrame += elapsedTime * fps;
            if (FrameTicked != null) FrameTicked(this, new EventArgs());

            // 最終フレームに達した時の処理
            if (CurrentFrame >= FinalFrame)
            {
                CurrentFrame = (actionAfterMotion == ActionAfterMotion.Replay) ? 1.0e-3f : FinalFrame;
                if (MotionFinished != null) MotionFinished(this, actionAfterMotion);
            }
        }

        /// <summary>
        /// IMotionProviderメンバーの実装
        /// </summary>
        public void Start(float frame, ActionAfterMotion actionAfterMotion)
        {
            if (frame > FinalFrame)
            {
                throw new InvalidOperationException("最終フレームを超えた場所から再生を求められました。");
            }
            CurrentFrame = frame;
            this.actionAfterMotion = actionAfterMotion;
            isPlaying = true;
        }

        /// <summary>
        /// IMotionProviderメンバーの実装
        /// </summary>
        public void Stop()
        {
            isPlaying = false;
        }

        /// <summary>
        /// IMotionProviderメンバーの実装
        /// </summary>
        public bool IsAttached
        {
            get { return isAttached; }
        }
    }
}
