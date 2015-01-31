using System;
using System.Collections.Generic;
using MMDFileParser.PMXModelParser;
using SlimDX;

namespace MMF.Bone
{
    /// <summary>
    ///     ボーンクラス
    /// </summary>
    public class PMXBone : IBone
    {

        private readonly ISkinningProvider skinning;

        /// <summary>
        ///     ボーンのインデックス
        /// </summary>
        public int BoneIndex;

        /// <summary>
        ///     ボーン名
        /// </summary>
        public string BoneName;

        /// <summary>
        ///     子ボーン
        /// </summary>
        public List<PMXBone> Children = new List<PMXBone>();

        public Vector3 DefaultLocalX;
        public Vector3 DefaultLocalY;
        public Vector3 DefaultLocalZ;

        /// <summary>
        ///     親ボーン
        /// </summary>
        public PMXBone Parent;

        /// <summary>
        ///     元々のローカル位置ベクタ
        /// </summary>
        public Vector3 Position;

        /// <summary>
        ///     平行移動位置ベクタ
        /// </summary>
        public Vector3 Translation { get; set; }

        public bool isLocalAxis;

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="bones"></param>
        /// <param name="index"></param>
        /// <param name="layer"></param>
        /// <param name="skinning"></param>
        public PMXBone(List<BoneData> bones, int index, int layer, ISkinningProvider skinning)
        {
            this.skinning = skinning;
            BoneData me = bones[index]; //このボーン
            skinning.Bone[index] = this;
            BoneIndex = index;
            Position = me.Position;
            BoneName = me.BoneName;
            
            isLocalAxis = me.isLocalAxis;
            PhysicsOrder = me.transformAfterPhysics ? PhysicsOrder.After : PhysicsOrder.Before;
            Layer = layer;
            if (isLocalAxis)
            {
                DefaultLocalX = me.DimentionXDirectionVector;
                DefaultLocalY = Vector3.Cross(me.DimentionZDirectionVector, DefaultLocalX);
                DefaultLocalZ = Vector3.Cross(DefaultLocalX, DefaultLocalY);
            }
            if (me.isIK) //IKボーンの場合
            {
                skinning.IkBone.Add(this);
                isIK = true;
                RotationLimited = me.IKLimitedRadian;
                targetBoneIndex = me.IKTargetBoneIndex;
                Iterator = me.IKLoopNumber;
                foreach (IkLinkData ikLink in me.ikLinks)
                {
                    ikLinks.Add(new IkLink(skinning, ikLink));
                }
            }
            isRotateProvided = me.isRotateProvided;
            isMoveProvided = me.isMoveProvided;
            if (me.ProvidedParentBoneIndex == -1)
            {
                isRotateProvided = isMoveProvided = false;
            }
            if (isMoveProvided || isRotateProvided)
            {
                ProvideParentBone = me.ProvidedParentBoneIndex;
                ProvidedRatio = me.ProvidedRatio;
            }
            else
            {
                ProvideParentBone = -1;
            }
            for (int i = 0; i < bones.Count; i++)
            {
                BoneData bone = bones[i];
                if (bone.ParentBoneIndex == index)
                {
                    PMXBone child = new PMXBone(bones, i, layer + 1, skinning);
                    AddChild(child);
                }
            }
        }

        /// <summary>
        ///     回転行列
        /// </summary>
        public Quaternion Rotation
        {
            get { return rotation; }
            set
            {
                rotation = value;
                rotation.Normalize();
            }
        }

        /// <summary>
        ///     変形階層
        /// </summary>
        public int Layer { get; private set; }

        /// <summary>
        ///     物理順
        /// </summary>
        public PhysicsOrder PhysicsOrder { get; private set; }


        public Vector3 LocalX
        {
            get { return Vector3.TransformNormal(DefaultLocalX, Matrix.RotationQuaternion(Rotation)); }
        }

        public Vector3 LocalY
        {
            get { return Vector3.TransformNormal(DefaultLocalY, Matrix.RotationQuaternion(Rotation)); }
        }

        public Vector3 LocalZ
        {
            get { return Vector3.TransformNormal(DefaultLocalZ, Matrix.RotationQuaternion(Rotation)); }
        }


        /// <summary>
        ///     ローカルポーズ
        /// </summary>
        public Matrix LocalPose { get; set; }

        /// <summary>
        ///     グローバルポーズ
        /// </summary>
        public Matrix GlobalPose { get; private set; }

        #region IKに関する項目

        private readonly int targetBoneIndex;

        /// <summary>
        ///     演算処理の繰り返し数
        /// </summary>
        public int Iterator;

        /// <summary>
        ///     １回あたりの演算の制限回転量
        /// </summary>
        public float RotationLimited;

        /// <summary>
        ///     IKリンクのリスト
        /// </summary>
        public List<IkLink> ikLinks = new List<IkLink>();

        /// <summary>
        ///     このボーンがIKボーンかどうか
        /// </summary>
        public bool isIK = false;

        private Quaternion rotation;

        public PMXBone IkTargetBone
        {
            get { return skinning.Bone[targetBoneIndex]; }
        }

        #endregion

        #region 付与に関する項目

        public bool isMoveProvided { get; private set; }

        public bool isRotateProvided { get; private set; }

        public int ProvideParentBone { get; private set; }

        public float ProvidedRatio { get; private set; }

        #endregion

        /// <summary>
        ///     子関節を追加します。
        /// </summary>
        /// <param name="child">追加する子関節</param>
        public void AddChild(PMXBone child)
        {
            Children.Add(child);
            child.Parent = this;
        }

        public void UpdateGrobalPose()
        {
            LocalPose = Matrix.Translation(-Position)*Matrix.RotationQuaternion(Rotation)*Matrix.Translation(Translation)*Matrix.Translation(Position);

            GlobalPose = LocalPose;
            if (Parent != null) //この要素がルートではないとき
            {
                GlobalPose *= Parent.GlobalPose;
            }
            // Parallel.ForEach(Children, (item) => item.UpdateGrobalPose());
            foreach (var child in Children)
                child.UpdateGrobalPose();
        }
    }
}