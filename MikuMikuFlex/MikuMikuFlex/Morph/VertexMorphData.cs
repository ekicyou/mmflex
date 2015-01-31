using System.Collections.Generic;
using MMDFileParser.PMXModelParser;
using MMDFileParser.PMXModelParser.MorphOffset;

namespace MMF.Morph
{
    public class VertexMorphData
    {
        
        public List<VertexMorphOffset> MorphOffsets = new List<VertexMorphOffset>();

        public VertexMorphData(MorphData morphData)
        {
            foreach (MorphOffsetBase morphOffsetBase in morphData.MorphOffsetes)
            {
               MorphOffsets.Add((VertexMorphOffset)morphOffsetBase);
            }
        }
    }
}
