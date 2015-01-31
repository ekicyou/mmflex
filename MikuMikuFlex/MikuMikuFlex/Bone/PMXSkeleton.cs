using System;
using System.Collections.Generic;
using MMDFileParser.PMXModelParser;
using MMF.Model;
using MMF.Morph;
using SlimDX;
using SlimDX.Direct3D11;
using Debug=System.Diagnostics.Debug;

namespace MMF.Bone
{
	public class PMXSkeleton : ISkinningProvider
    {
        /// <summary>
        ///     ボーンそれぞれのグローバルポーズ
        /// </summary>
        public Matrix[] GlobalBonePose;

        /// <summary>
        /// ボーンが変形された後であることを表します。
        /// </summary>
	    public event EventHandler SkeletonUpdated = delegate { };

        /// <summary>
        ///     Ikを処理するインターフェース
        /// </summary>
        public ITransformUpdater IkProvider;


        public List<ITransformUpdater> KinematicsProviders { get; private set; }

        /// <summary>
        ///     ボーンのルート(ルートが2つ以上の場合があり)
        /// </summary>
        public List<PMXBone> RootBone = new List<PMXBone>();

        public PMXSkeleton(ModelData model)
        {
            
            //ボーンの数だけ初期化
            GlobalBonePose = new Matrix[model.BoneList.BoneCount];
            Bone = new PMXBone[model.BoneList.BoneCount];
            IkBone = new List<PMXBone>();
            //ボーンを読み込む
            LoadBones(model);
            BoneDictionary = new Dictionary<string, PMXBone>();
            foreach (var bone in Bone)
            {
                if (BoneDictionary.ContainsKey(bone.BoneName))
                {

                    int i = 0;
                    do
                    {
                        i++;
                    } while (BoneDictionary.ContainsKey(bone.BoneName + i.ToString()));
                    BoneDictionary.Add(bone.BoneName+i.ToString(),bone);
                    Debug.WriteLine("ボーン名{0}は重複しています。自動的にボーン名{1}と読み替えられました。",bone.BoneName,bone.BoneName+i);
                }else
                BoneDictionary.Add(bone.BoneName,bone);
            }
            KinematicsProviders = new List<ITransformUpdater>();
            IkProvider = new CCDIK(IkBone);
            KinematicsProviders.Add(IkProvider);
            KinematicsProviders.Add(new BestrowKinematicsProvider(Bone));
            if (Bone.Length > 512)
            {
                throw new InvalidOperationException("MMFでは現在512以上のボーンを持つモデルについてサポートしていません。\nただし、Resource\\Shader\\DefaultShader.fx内のボーン変形行列の配列float4x4 BoneTrans[512]:BONETRANS;の要素数を拡張しこの部分をコメントアウトすれば暫定的に利用することができるかもしれません。");
            }
        }


        /// <summary>
        ///     ボーン(インデックス順)
        /// </summary>
        public PMXBone[] Bone { get; set; }

	    public Dictionary<string, PMXBone> BoneDictionary { get; private set; }

	    /// <summary>
        ///     Ikボーンのリスト
        /// </summary>
        public List<PMXBone> IkBone { get; set; }

        /// <summary>
        ///     エフェクトに、現在のボーン情報を渡します。
        /// </summary>
        /// <param name="effect"></param>
        public void ApplyEffect(Effect effect)
        {
            effect.GetVariableBySemantic("BONETRANS").AsMatrix().SetMatrixArray(GlobalBonePose);
        }

	    /// <summary>
	    ///     スキニング全体を更新します
	    /// </summary>
	    /// <param name="morphManager"></param>
	    public virtual void UpdateSkinning(IMorphManager morphManager)
        {
            ResetAllBoneTransform();
            UpdateGlobal();
            foreach (ITransformUpdater kinematicsProvider in KinematicsProviders)
            {
                if (kinematicsProvider.UpdateTransform())
                {
                    UpdateGlobal();
                    //ResetAllBoneTransform();// BUG これなんでいれたっけ？
                }
            }
	        foreach (var pmxBone in RootBone)
	        {
	            pmxBone.UpdateGrobalPose();
	        }
            SkeletonUpdated(this,new EventArgs());
            for (int i = 0; i < Bone.Length; i++)
            {
                GlobalBonePose[Bone[i].BoneIndex] = Bone[i].GlobalPose;
            }
        }

        /// <summary>
        ///     すべての回転・移動を元に戻します。
        /// </summary>
        public void ResetAllBoneTransform()
        {
            foreach (PMXBone item in Bone)
            {
                item.Rotation = Quaternion.Identity;
                item.Translation = Vector3.Zero;
            }
        }

        private void LoadBones(ModelData model)
        {
            for (int i = 0; i < model.BoneList.BoneCount; i++)
            {
                if (model.BoneList.Bones[i].ParentBoneIndex == -1)
                {
                    RootBone.Add(new PMXBone(model.BoneList.Bones, i, 0, this));
                }
            }
            BoneComparer comparer = new BoneComparer(model.BoneList.Bones.Count);
            IkBone.Sort(comparer);
            RootBone.Sort(comparer);
        }

        /// <summary>
        ///     現在の回転行列に基づき、ルートボーンからグローバルポーズを再計算します。
        /// </summary>
        protected void UpdateGlobal()
        {
            // Parallel.ForEach(RootBone, (item) => item.UpdateGrobalPose());
            foreach (var root in RootBone)
                root.UpdateGrobalPose();
        }

		public virtual void Dispose() {}
    }
}