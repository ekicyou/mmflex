using System.Collections.Generic;

namespace MMF.Bone
{
    /// <summary>
    ///     ボーンのソート用クラス
    /// </summary>
    internal class BoneComparer : IComparer<PMXBone>
    {
        public BoneComparer(int boneCount)
        {
            BoneCount = boneCount;
        }

        public int BoneCount { get; private set; }

        #region IComparer<Bone> メンバー

        /// <summary>
        ///     ボーンの計算順序順に並べ替える
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        public int Compare(PMXBone x, PMXBone y)
        {
            //後であればあるほどスコアが大きくなるように計算する
            int xScore = 0;
            int yScore = 0;
            if (x.PhysicsOrder == PhysicsOrder.After)
            {
                xScore += BoneCount*BoneCount;
            }
            if (y.PhysicsOrder == PhysicsOrder.After)
            {
                yScore += BoneCount*BoneCount;
            }
            xScore += BoneCount*x.Layer;
            yScore += BoneCount*y.Layer;
            xScore += x.BoneIndex;
            yScore += y.BoneIndex;
            return xScore - yScore;
        }

        #endregion
    }
}