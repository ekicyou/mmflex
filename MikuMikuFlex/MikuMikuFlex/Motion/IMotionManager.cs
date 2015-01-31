using System;
using System.Collections.Generic;
using System.IO;
using MMDFileParser.PMXModelParser;
using MMF.Bone;
using MMF.Model;
using MMF.Morph;

namespace MMF.Motion
{
    /// <summary>
    ///     動かせるモデル用
    /// </summary>
    public interface IMotionManager:ITransformUpdater
    {

        IMotionProvider CurrentMotionProvider { get; }

        /// <summary>
        /// 現在再生中のモーションのフレーム位置(秒単位)を取得します。
        /// </summary>
        float CurrentFrame { get; }

        /// <summary>
        /// 前回のフレームからどれだけ時間がかかったか(秒単位)取得します
        /// </summary>
        float ElapsedTime { get; }

        /// <summary>
        /// ファイル名とモーションプロバイダの対応表
        /// </summary>
        List<KeyValuePair<string, IMotionProvider>> SubscribedMotionMap { get; }
        
        /// <summary>
        ///     初期化処理
        /// </summary>
        /// <param name="skinning"></param>
        void Initialize(ModelData model,IMorphManager morph, ISkinningProvider skinning, IBufferManager bufferManager);

        /// <summary>
        ///     ファイルからモーションを追加します。
        /// </summary>
        /// <param name="filePath"></param>
        /// <param name="ignoreParent"></param>
        /// <returns>追加されたモーションのインデックス</returns>
        IMotionProvider AddMotionFromFile(string filePath,bool ignoreParent);

        /// <summary>
        ///     モーションを適用し動かす。
        /// </summary>
        /// <param name="index"></param>
        void ApplyMotion(IMotionProvider provider, int startFrame = 0, ActionAfterMotion setting = ActionAfterMotion.Nothing);

        void StopMotion(bool toIdentity=false);


        event EventHandler<ActionAfterMotion> MotionFinished;


        /// <summary>
        ///     モーションのリストがアップデートされたときに呼び出されます。
        /// </summary>
        event EventHandler MotionListUpdated;

        IMotionProvider AddMotionFromStream(string fileName, Stream stream, bool ignoreParent);
    }
}