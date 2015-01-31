using System.Collections.Generic;
using MMDFileParser.PMXModelParser;
using MMDFileParser.PMXModelParser.MorphOffset;

namespace MMF.Morph
{
    internal class BoneMorphData
    {
        public List<BoneMorphOffset> BoneMorphs = new List<BoneMorphOffset>();

        public BoneMorphData(MorphData morphData)
        {
            foreach (MorphOffsetBase morphOffsetBase in morphData.MorphOffsetes)
            {
                BoneMorphs.Add((BoneMorphOffset) morphOffsetBase);
            }
        }
    }
}