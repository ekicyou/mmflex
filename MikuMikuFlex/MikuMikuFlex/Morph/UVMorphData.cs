using System.Collections.Generic;
using MMDFileParser.PMXModelParser;
using MMDFileParser.PMXModelParser.MorphOffset;

namespace MMF.Morph
{
    public class UVMorphData
    {
        public List<UVMorphOffset> MorphOffsets=new List<UVMorphOffset>();

        public UVMorphData(MorphData morphData)
        {
            foreach (MorphOffsetBase morphOffsetBase in morphData.MorphOffsetes)
            {
                MorphOffsets.Add((UVMorphOffset)morphOffsetBase);
            }
        }
    }
}