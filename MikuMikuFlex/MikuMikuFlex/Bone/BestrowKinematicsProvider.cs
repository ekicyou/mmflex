using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMF.Morph;
using MMF.Utility;
using SlimDX;

namespace MMF.Bone
{
    class BestrowKinematicsProvider : ITransformUpdater
    {
        /// <summary>
        /// ボーンの配列
        /// </summary>
        private HierarchicalOrderCollection<PMXBone> bones;

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="bones">ボーンの配列</param>
        public BestrowKinematicsProvider(PMXBone[] bones)
        {
            this.bones =new HierarchicalOrderCollection<PMXBone>(bones,new BestrowKinematicsOrderSolver());
        }

        /// <summary>
        /// ITransformUpdaterのメンバーの実装
        /// </summary>
        public bool UpdateTransform()
        {
            foreach (var pmxBone in bones)
            {
                if (pmxBone.isMoveProvided)
                {
                    var pp = bones[pmxBone.ProvideParentBone];
                    pmxBone.Translation += Vector3.Lerp(Vector3.Zero, pp.Translation, pmxBone.ProvidedRatio);
                }
                if (pmxBone.isRotateProvided)
                {
                    var pp = bones[pmxBone.ProvideParentBone];
                    pmxBone.Rotation *= Quaternion.Slerp(Quaternion.Identity, pp.Rotation, pmxBone.ProvidedRatio);
                }
            }
            return true;
        }

        private class BestrowKinematicsOrderSolver : HierarchicalOrderSolver<PMXBone>
        {
            public int getParentIndex(PMXBone child)
            {
                if (!child.isMoveProvided && child.isRotateProvided) return -1;
                return child.ProvideParentBone;
            }

            public int getIndex(PMXBone target)
            {
                return target.BoneIndex;
            }
        }
 
    }
}
