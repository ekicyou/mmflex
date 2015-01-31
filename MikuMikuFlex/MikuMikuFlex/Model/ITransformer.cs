using SlimDX;

namespace MMF.Model
{
    public interface ITransformer
    {
        /// <summary>
        ///     モデルのワールド座標での位置を表します。
        /// </summary>
        Vector3 Position { get; set; }

        /// <summary>
        ///     モデルのワールド空間において向いている向きを表します。
        /// </summary>
        Vector3 Foward { get; set; }

        /// <summary>
        ///     ワールド空間において上の向きを表します。
        /// </summary>
        Vector3 Top { get; set; }

        /// <summary>
        ///     モデルのワールド空間においての回転を表します。
        /// </summary>
        Quaternion Rotation { get; set; }

        /// <summary>
        ///     モデルの倍率を表します
        /// </summary>
        Vector3 Scale { get; set; }

        /// <summary>
        ///     初期状態での上方向を表します
        /// </summary>
        Vector3 InitialTop { get; }

        /// <summary>
        ///     初期状態での向いている方向を表します。
        /// </summary>
        Vector3 InitialFoward { get; }

        /// <summary>
        ///     モデルの位置を最初の状態に戻します
        /// </summary>
        void Reset();

        /// <summary>
        /// モデルの変換行列
        /// </summary>
        Matrix LocalTransform { get; }
    }
}