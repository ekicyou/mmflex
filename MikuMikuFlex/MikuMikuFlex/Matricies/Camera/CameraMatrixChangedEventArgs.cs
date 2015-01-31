using System;

namespace MMF.Matricies.Camera
{
    /// <summary>
    ///     カメラの設定が変更された時のイベントアーギュメント
    /// </summary>
    public class CameraMatrixChangedEventArgs : EventArgs
    {
        /// <summary>
        ///     コンストラクタ
        /// </summary>
        /// <param name="type">変えられたカメラの内容</param>
        public CameraMatrixChangedEventArgs(CameraMatrixChangedVariableType type)
        {
            ChangedType = type;
        }

        /// <summary>
        ///     変えられた内容
        /// </summary>
        public CameraMatrixChangedVariableType ChangedType { get; private set; }
    }
}