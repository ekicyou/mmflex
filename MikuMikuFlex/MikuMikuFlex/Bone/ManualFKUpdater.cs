using System;
using System.Collections.Generic;
using MMF.Model;
using MMF.Model.PMX;
using MMF.Morph;
using SlimDX;

namespace MMF.Bone
{
    public class ManualTransformUpdater:ITransformUpdater
    {
        private Dictionary<string, BoneTransformer> updaters = new Dictionary<string, BoneTransformer>();

        private Dictionary<string, MorphTransformer> morphUpdaters = new Dictionary<string, MorphTransformer>();

        private PMXModel model;

        public ManualTransformUpdater(PMXModel model)
        {
            this.model = model;
        }

        /// <summary>
        ///  ITransformUpdaterメンバーの実装
        /// </summary>
        public bool UpdateTransform()
        {
            var boneDictionary = model.Skinning.BoneDictionary;
            var morphManager = model.Morphmanager;
            foreach (var boneTransformer in updaters)
            {
                var bone = boneDictionary[boneTransformer.Key];
                bone.Rotation *= boneTransformer.Value.Rotation;
                bone.Translation += boneTransformer.Value.Translation;
            }
            foreach (var morphTransformer in morphUpdaters)
            {
                morphManager.ApplyMorphProgress(morphTransformer.Value.MorphValue,morphTransformer.Key);
            }
            return true;
        }

        public BoneTransformer getBoneTransformer(string boneName)
        {
            BoneTransformer transformer;
            if(!model.Skinning.BoneDictionary.ContainsKey(boneName))throw new InvalidOperationException("そのような名前のボーンは存在しません。");
            if (updaters.ContainsKey(boneName)) return updaters[boneName];
            else transformer=new BoneTransformer(boneName,Quaternion.Identity,Vector3.Zero);
            updaters.Add(transformer.BoneName,transformer);
            return transformer;
        }

        public MorphTransformer getMorphTransformer(string morphName)
        {
            MorphTransformer transformer;
            if (morphUpdaters.ContainsKey(morphName)) return morphUpdaters[morphName];
            else transformer=new MorphTransformer(morphName);
            morphUpdaters.Add(transformer.MorphName,transformer);
            return transformer;
        }
    }

    public class MorphTransformer
    {
        public string MorphName { get; private set; }

        public float MorphValue { get; set; }

        public MorphTransformer(string morphName)
        {
            MorphName = morphName;
        }
    }
}
