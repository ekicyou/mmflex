using System.Collections.Generic;
using MMDFileParser.PMXModelParser;
using MMDFileParser.PMXModelParser.MorphOffset;

namespace MMF.Morph
{
    internal class MaterialMorphData
    {
        public List<MaterialMorphOffset> Morphoffsets=new List<MaterialMorphOffset>();

        public MaterialMorphData(MorphData morph)
        {
            foreach (MorphOffsetBase morphOffsetBase in morph.MorphOffsetes)
            {
                Morphoffsets.Add((MaterialMorphOffset)morphOffsetBase);
            }
        }
    }
}