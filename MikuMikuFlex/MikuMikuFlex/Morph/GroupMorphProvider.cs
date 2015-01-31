using System;
using System.Collections.Generic;
using MMDFileParser.PMXModelParser;
using MMDFileParser.PMXModelParser.MorphOffset;
using MMF.Model;
using MMF.Model.PMX;
using MMF.Motion;

namespace MMF.Morph
{
    public class GroupMorphProvider:IMorphProvider
    {
        public GroupMorphProvider(PMXModel model,IMorphManager morph)
        {
            this.morphManager = morph;
            int i = 0;
            foreach (MorphData morphData in model.Model.MorphList.Morphes)
            {
                if (morphData.type == MorphType.Group)
                {
                    Morphs.Add(morphData.MorphName,new GroupMorphData(morphData));
                }
                morphNameList.Add(i,morphData.MorphName);
                i++;
            }
        }

        private IMorphManager morphManager;

        private Dictionary<int,string> morphNameList=new Dictionary<int, string>(); 

        public Dictionary<string,GroupMorphData> Morphs=new Dictionary<string, GroupMorphData>(); 

        public void ApplyMorphProgress(float frameNumber, IEnumerable<MorphMotion> morphMotions)
        {
            foreach (var morphMotion in morphMotions)
            {
                SetMorphProgress(morphMotion.GetMorphValue(frameNumber), morphMotion.MorphName);
            }
        }

        public bool ApplyMorphProgress(float progress, string morphName)
        {
            return SetMorphProgress(progress, morphName);
        }

        public void UpdateFrame()
        {
            
        }

        private bool SetMorphProgress(float progress, string morphName)
        {
            if (!Morphs.ContainsKey(morphName)) return false;
            GroupMorphData data = Morphs[morphName];
            foreach (GroupMorphOffset groupMorphOffset in data.MorphOffsets)
            {
                string targetMorph = morphNameList[groupMorphOffset.MorphIndex];
                if(morphName.Equals(targetMorph))throw new InvalidOperationException("グループモーフに自身のモーフが指定されています。");
                morphManager.ApplyMorphProgress(progress*groupMorphOffset.MorphRatio,targetMorph);
            }
            return true;
        }
    }

    public class GroupMorphData
    {
        public List<GroupMorphOffset> MorphOffsets=new List<GroupMorphOffset>();

        public GroupMorphData(MorphData data)
        {
            foreach (MorphOffsetBase morphOffsetBase in data.MorphOffsetes)
            {
                MorphOffsets.Add((GroupMorphOffset)morphOffsetBase);
            }
        }
    }
}
