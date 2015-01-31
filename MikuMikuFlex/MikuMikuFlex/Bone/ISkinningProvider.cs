using System;
using System.Collections.Generic;
using MMF.Model;
using MMF.Morph;
using SlimDX.Direct3D11;

namespace MMF.Bone
{
    public interface ISkinningProvider : System.IDisposable
    {
        PMXBone[] Bone { get; }

        Dictionary<string,PMXBone> BoneDictionary { get; }
        
        List<PMXBone> IkBone { get; }

        List<ITransformUpdater> KinematicsProviders { get; }

        /// <summary>
        ///     モーションデータをエフェクトに適用する際に呼び出します。
        /// </summary>
        /// <param name="effect"></param>
        void ApplyEffect(Effect effect);

        /// <summary>
        ///     フレームごとにスキニングのデータを更新するために呼び出します。
        /// </summary>
        /// <param name="morphManager"></param>
        void UpdateSkinning(IMorphManager morphManager);

        /// <summary>
        ///     関節のすべての回転をリセットします。
        /// </summary>
        void ResetAllBoneTransform();

        event EventHandler SkeletonUpdated;
    }
}