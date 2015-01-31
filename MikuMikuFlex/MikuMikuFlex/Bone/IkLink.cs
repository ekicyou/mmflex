using System;
using MMF.Utility;
using SlimDX;

namespace MMF.Bone
{
    /// <summary>
    ///     IKリンクに関わるデータ
    /// </summary>
    public class IkLink
    {
        private readonly int index;
        private readonly ISkinningProvider skinning;

        /// <summary>
        ///     回転量制限があるかどうか
        /// </summary>
        public bool isLimited = false;

        /// <summary>
        ///     最大回転量(X,Y,Z)
        /// </summary>
        public Vector3 maxRot;

        /// <summary>
        ///     最小回転量(X,Y,Z)
        /// </summary>
        public Vector3 minRot;

        public int loopCount;

        public IkLink(ISkinningProvider skinning,MMDFileParser.PMXModelParser.IkLinkData linkData)
        {
            this.skinning = skinning;
            this.index = linkData.LinkBoneIndex;
            Vector3 maxVec = linkData.MinimumRadian;
            Vector3 minVec = linkData.MaximumRadian;
            //minとmaxを正しく読み込む
            minRot = new Vector3(Math.Min(maxVec.X, minVec.X), Math.Min(maxVec.Y, minVec.Y),
                Math.Min(maxVec.Z, minVec.Z));
            maxRot = new Vector3(Math.Max(maxVec.X, minVec.X), Math.Max(maxVec.Y, minVec.Y),
                Math.Max(maxVec.Z, minVec.Z));
            maxRot = Vector3.Clamp(maxRot, CGHelper.EularMinimum, CGHelper.EularMaximum);
            minRot = Vector3.Clamp(minRot, CGHelper.EularMinimum, CGHelper.EularMaximum);
            isLimited = linkData.isRotateLimited;
        }

        /// <summary>
        ///     ikリンクボーン
        /// </summary>
        public PMXBone ikLinkBone
        {
            get { return skinning.Bone[index]; }
        }
    }
}