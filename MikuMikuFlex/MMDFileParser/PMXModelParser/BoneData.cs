using SlimDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class BoneData
    {
        internal static BoneData getBone(FileStream fs, Header header)
        {
            BoneData bone = new BoneData();
            bone.ikLinks = new List<IkLinkData>();
            bone.BoneName = ParserHelper.getTextBuf(fs, header.Encode);
            bone.BoneName_En = ParserHelper.getTextBuf(fs, header.Encode);
            bone.Position = ParserHelper.getFloat3(fs);
            bone.ParentBoneIndex = ParserHelper.getIndex(fs, header.BoneIndexSize);
            bone.TranslationLevel = ParserHelper.getInt(fs);

            byte[] flag = new byte[2];
            flag[0] = ParserHelper.getByte(fs); flag[1] = ParserHelper.getByte(fs);
            Int16 flagnum = BitConverter.ToInt16(flag, 0);
            bone.boneConnectTo = ParserHelper.isFlagEnabled(flagnum, 0x0001) ? BoneConnectTo.Bone : BoneConnectTo.PositionOffset;
            bone.canRotate = ParserHelper.isFlagEnabled(flagnum, 0x0002);
            bone.canMove = ParserHelper.isFlagEnabled(flagnum, 0x0004);
            bone.isVisible = ParserHelper.isFlagEnabled(flagnum, 0x0008);
            bone.canOperate = ParserHelper.isFlagEnabled(flagnum, 0x0010);
            bone.isIK = ParserHelper.isFlagEnabled(flagnum, 0x0020);
            bone.localProvideTo = ParserHelper.isFlagEnabled(flagnum, 0x0080) ? LocalProvideTo.ParentLocalTransformValue : LocalProvideTo.UserTransformValue;
            bone.isRotateProvided = ParserHelper.isFlagEnabled(flagnum, 0x0100);
            bone.isMoveProvided = ParserHelper.isFlagEnabled(flagnum, 0x0200);
            bone.isfixAxis = ParserHelper.isFlagEnabled(flagnum, 0x0400);
            bone.isLocalAxis = ParserHelper.isFlagEnabled(flagnum, 0x0800);
            bone.transformAfterPhysics = ParserHelper.isFlagEnabled(flagnum, 0x1000);
            bone.ParentTransform = ParserHelper.isFlagEnabled(flagnum, 0x2000);
            if (bone.boneConnectTo == BoneConnectTo.PositionOffset)
            {
                bone.PositionOffset = ParserHelper.getFloat3(fs);
            }
            else
            {
                bone.ConnectedBoneIndex = ParserHelper.getIndex(fs, header.BoneIndexSize);
            }
            if (bone.isRotateProvided || bone.isMoveProvided)
            {
                bone.ProvidedParentBoneIndex = ParserHelper.getIndex(fs, header.BoneIndexSize);
                bone.ProvidedRatio = ParserHelper.getFloat(fs);
            }
            if (bone.isfixAxis) bone.AxisDirectionVector = ParserHelper.getFloat3(fs);
            if (bone.isLocalAxis)
            {
                bone.DimentionXDirectionVector = ParserHelper.getFloat3(fs);
                bone.DimentionZDirectionVector = ParserHelper.getFloat3(fs);
            }
            if (bone.ParentTransform) bone.KeyValue = ParserHelper.getInt(fs);
            if (bone.isIK)
            {
                bone.IKTargetBoneIndex = ParserHelper.getIndex(fs, header.BoneIndexSize);
                bone.IKLoopNumber = ParserHelper.getInt(fs);
                bone.IKLimitedRadian = ParserHelper.getFloat(fs);
                bone.IKLinkCount = ParserHelper.getInt(fs);
                for (int i = 0; i < bone.IKLinkCount; i++)
                {
                    bone.ikLinks.Add(IkLinkData.getIKLink(fs, header));
                }
            }
            return bone;
        }


        public String BoneName { get; private set; }

        public String BoneName_En { get; private set; }

        public Vector3 Position { get; private set; }

        public int ParentBoneIndex { get; private set; }

        public int TranslationLevel { get; private set; }

        public BoneConnectTo boneConnectTo { get; private set; }

        public bool canRotate { get; private set; }

        public bool canMove { get; private set; }

        public bool isVisible { get; private set; }

        public bool canOperate { get; private set; }

        public bool isIK { get; private set; }

        public LocalProvideTo localProvideTo { get; private set; }

        public bool isRotateProvided { get; private set; }

        public bool isMoveProvided { get; private set; }

        public bool isfixAxis { get; private set; }

        public bool isLocalAxis { get; private set; }

        public bool transformAfterPhysics { get; private set; }

        public bool ParentTransform { get; private set; }

        public Vector3 PositionOffset { get; private set; }

        public int ConnectedBoneIndex { get; private set; }

        public int ProvidedParentBoneIndex { get; private set; }

        public float ProvidedRatio { get; private set; }

        public Vector3 AxisDirectionVector { get; private set; }

        public Vector3 DimentionXDirectionVector { get; private set; }

        public Vector3 DimentionZDirectionVector { get; private set; }

        public int KeyValue { get; private set; }

        public int IKTargetBoneIndex { get; private set; }

        public int IKLoopNumber { get; private set; }

        public float IKLimitedRadian { get; private set; }

        public int IKLinkCount { get; private set; }

        public List<IkLinkData> ikLinks { get; private set; }
    }

    public enum BoneConnectTo
    {
        PositionOffset, Bone
    }

    public enum LocalProvideTo
    {
        UserTransformValue, ParentLocalTransformValue
    }
}
