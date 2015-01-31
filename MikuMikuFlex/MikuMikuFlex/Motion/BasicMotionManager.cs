using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using MMDFileParser.PMXModelParser;
using MMF.Bone;
using MMF.Model;
using MMF.Morph;

namespace MMF.Motion
{
    /// <summary>
    ///     モーション管理クラス
    /// </summary>
    public class BasicMotionManager : IMotionManager
    {
        private readonly RenderContext context;

        private Stopwatch motionTimer;

        /// <summary>
        ///     スキニングに利用するインターフェース
        /// </summary>
        private ISkinningProvider skinningProvider;

        private IMorphManager morphManager;

        public BasicMotionManager(RenderContext context)
        {
            this.context = context;
        }

        private long lastTime { get; set; }

        /// <summary>
        /// ファイル名とモーションプロバイダの対応表
        /// </summary>
        public List<KeyValuePair<string, IMotionProvider>> SubscribedMotionMap { get; private set; }

        /// <summary>
        ///     現在再生中のモーションのindex
        /// </summary>
        public IMotionProvider CurrentMotionProvider { get; set; }

        /// <summary>
        ///     現在再生中のモーションのフレーム位置(秒単位)を取得します。
        /// </summary>
        public float CurrentFrame
        {
            get
            {
                if (CurrentMotionProvider == null) return float.NaN;
                return CurrentMotionProvider.CurrentFrame/context.Timer.MotionFramePerSecond;
            }
        }

        /// <summary>
        ///     前回のフレームからどれだけ時間がかかったか(ミリ秒単位)取得します
        /// </summary>
        public float ElapsedTime { get; private set; }

        /// <summary>
        ///     初期化
        /// </summary>
        /// <param name="skinning">スキニングに利用するインターフェース</param>
        public void Initialize(ModelData model,IMorphManager morph,ISkinningProvider skinning,IBufferManager bufferManager)
        {
            skinningProvider = skinning;
            motionTimer=new Stopwatch();
            motionTimer.Start();
            this.morphManager = morph;
            SubscribedMotionMap = new List<KeyValuePair<string, IMotionProvider>>();
        }

        /// <summary>
        /// ITransformUpdaterのメンバーの実装
        /// </summary>
        public bool UpdateTransform()
        {
            if (lastTime == 0)
            {
                lastTime = motionTimer.ElapsedMilliseconds;
            }
            else
            {
                long currentTime = motionTimer.ElapsedMilliseconds;
                ElapsedTime = currentTime - lastTime;
                if (CurrentMotionProvider != null) 
                    CurrentMotionProvider.Tick(context.Timer.MotionFramePerSecond, ElapsedTime / 1000f, morphManager);
                lastTime = currentTime;
            }
            return true;
        }


        /// <summary>
        ///     モーションをファイルから追加する
        /// </summary>
        /// <param name="filePath">ファイルパス</param>
        /// <param name="ignoreParent">すべての親を無視するか否か</param>
        /// <returns>モーションのindex</returns>
        public IMotionProvider AddMotionFromFile(string filePath,bool ignoreParent)
        {
            // モーションプロバイダを作成する。ファイルの拡張子に応じて適切なクラスを割り当てる
            IMotionProvider motion;
            var extension = System.IO.Path.GetExtension(filePath);
            if (String.Compare(extension, ".vmd", true) == 0) motion = new MMDMotion(filePath, ignoreParent);
            else if (String.Compare(extension, ".vme", true) == 0) motion = new MMDMotionForVME(filePath, ignoreParent);
            else throw new Exception("ファイルが不適切です！");

            motion.AttachMotion(skinningProvider.Bone);
            motion.MotionFinished += motion_MotionFinished;
            SubscribedMotionMap.Add(new KeyValuePair<string, IMotionProvider>(filePath, motion));
            if (MotionListUpdated != null) MotionListUpdated(this, new EventArgs());
            return motion;
        }

        /// <summary>
        ///     指定したモーションを再生する
        /// </summary>
        /// <param name="id">モーションのid</param>
        /// <param name="startFrame">最初のフレーム</param>
        /// <param name="setting">終了後の挙動</param>
        public void ApplyMotion(IMotionProvider motionProvider, int startFrame = 0,
            ActionAfterMotion setting = ActionAfterMotion.Nothing)
        {
            if (CurrentMotionProvider != null) CurrentMotionProvider.Stop();
            motionProvider.Start(startFrame, setting);
            CurrentMotionProvider = motionProvider;
        }

        /// <summary>
        ///     モーション再生を終了する
        /// </summary>
        public void StopMotion(bool toIdentity=false)
        {
            if(CurrentMotionProvider!=null)CurrentMotionProvider.Stop();
            if (toIdentity)
            {
                CurrentMotionProvider = null;
            }
        }

        /// <summary>
        ///     モーションが終了したことを通知する
        /// </summary>
        public event EventHandler<ActionAfterMotion> MotionFinished;

        /// <summary>
        ///     モーションのリストが更新されたことを通知する
        /// </summary>
        public event EventHandler MotionListUpdated;

        public IMotionProvider AddMotionFromStream(string fileName, Stream stream,bool ignoreParent)
        {
            IMotionProvider motion = new MMDMotion(stream, ignoreParent);
            motion.MotionFinished += motion_MotionFinished;
            SubscribedMotionMap.Add(new KeyValuePair<string, IMotionProvider>(fileName, motion));
            if (MotionListUpdated != null)MotionListUpdated(this, new EventArgs());
            return motion;
        }

        private void motion_MotionFinished(object owner, ActionAfterMotion obj)
        {
            if (MotionFinished != null) MotionFinished(this, obj);
        }
    }
}