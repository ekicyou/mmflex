using System;
using MMF.Bone;
using MMF.Morph;

namespace MMF.Motion
{
    public interface IMotionProvider
    {
        /// <summary>
        /// アタッチ済みか否か
        /// </summary>
        bool IsAttached { get; }

        /// <summary>
        ///     現在のフレーム
        /// </summary>
        float CurrentFrame { get; set; }

        /// <summary>
        ///     このモーションにおける最終フレーム
        /// </summary>
        int FinalFrame { get;}

        /// <summary>
        /// モーションをアタッチする
        /// </summary>
        /// <param name="bones">ボーン配列</param>
        void AttachMotion(PMXBone[] bones);

        /// <summary>
        ///     モーションを再生します。
        /// </summary>
        /// <param name="frame">再生フレームの最初</param>
        /// <param name="action">モーション終了後の挙動</param>
        void Start(float frame, ActionAfterMotion action);


        /// <summary>
        ///     モーションを停止します。
        /// </summary>
        void Stop();

        /// <summary>
        /// モーションを１フレーム進めます。
        /// </summary>
        /// <param name="fps">秒間フレーム数</param>
        /// <param name="elapsedTime">前回のフレームからどれだけ時間がかかったか[秒]</param>
        /// <param name="morphManager">モーフの管理クラス</param>
        void Tick(int fps, float elapsedTime, IMorphManager morphManager);

        /// <summary>
        ///     モーションが終了したことを表します。
        /// </summary>
        event EventHandler<ActionAfterMotion> MotionFinished;

        event EventHandler<EventArgs> FrameTicked;
    }

    /// <summary>
    ///     モーション再生終了後の挙動を指定する列挙体
    /// </summary>
    public enum ActionAfterMotion
    {
        Nothing,
        Replay
    }
}