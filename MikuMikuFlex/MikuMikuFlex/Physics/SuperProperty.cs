using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using BulletSharp;

namespace MMF.Physics
{
    /// <summary>
    /// 物理演算を超越した特性
    /// </summary>
    internal class SuperProperty
    {
        public bool kinematic { private set; get; }
        public CollisionFilterGroups group { private set; get; }
        public CollisionFilterGroups mask { private set; get; }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="kinematic">物理演算の影響を受けないKinematic剛体か否か</param>
        /// <param name="group">自身の衝突グループNo.</param>
        /// <param name="mask">自身と衝突する他の衝突グループNo.</param>
        public SuperProperty(bool kinematic = false, CollisionFilterGroups group = CollisionFilterGroups.DefaultFilter, CollisionFilterGroups mask = CollisionFilterGroups.AllFilter)
        {
            this.kinematic = kinematic;
            this.group = group;
            this.mask = mask;
        }
    }

}
