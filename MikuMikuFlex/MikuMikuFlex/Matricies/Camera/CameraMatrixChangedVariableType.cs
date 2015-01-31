namespace MMF.Matricies.Camera
{
    /// <summary>
    ///     カメラの変更内容に利用する
    ///     カメラの変数の種類
    /// </summary>
    public enum CameraMatrixChangedVariableType
    {
        /// <summary>
        ///     位置
        /// </summary>
        Position,

        /// <summary>
        ///     注視点
        /// </summary>
        LookAt,

        /// <summary>
        ///     上方向ベクトル
        /// </summary>
        Up
    }
}