using MMDFileParser.PMXModelParser.MorphOffset;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class MorphData
    {
        internal static MorphData getMorph(FileStream fs,Header header)
        {
            MorphData morph = new MorphData();
            morph.MorphOffsetes = new List<MorphOffsetBase>();
            morph.MorphName = ParserHelper.getTextBuf(fs, header.Encode);
            morph.MorphName_En = ParserHelper.getTextBuf(fs, header.Encode);
            morph.OperationPanel = ParserHelper.getByte(fs);
            byte Morphtype = ParserHelper.getByte(fs);
            morph.MorphOffsetCount= ParserHelper.getInt(fs);
            for (int i = 0; i < morph.MorphOffsetCount; i++)
            {
                switch (Morphtype)
                {
                    case 0:
                        //Group Morph
                        morph.type = MorphType.Group;
                        morph.MorphOffsetes.Add(GroupMorphOffset.getGroupMorph(fs, header));
                        break;
                    case 1:
                        //Vertex Morph
                        morph.type = MorphType.Vertex;
                        morph.MorphOffsetes.Add(VertexMorphOffset.getVertexMorph(fs, header));
                        break;
                    case 2:
                        morph.type = MorphType.Bone;
                        morph.MorphOffsetes.Add(BoneMorphOffset.getBoneMorph(fs, header));
                        break;
                        //3~7はすべてUVMorph
                    case 3:
                        morph.type = MorphType.UV;
                        morph.MorphOffsetes.Add(UVMorphOffset.getUVMorph(fs,header,MorphType.UV));
                        break;
                    case 4:
                        morph.type = MorphType.UV_Additional1;
                        morph.MorphOffsetes.Add(UVMorphOffset.getUVMorph(fs,header,MorphType.UV_Additional1));
                        break;
                    case 5:
                        morph.type = MorphType.UV_Additional2;
                        morph.MorphOffsetes.Add(UVMorphOffset.getUVMorph(fs,header,MorphType.UV_Additional2));
                        break;
                    case 6:
                        morph.type = MorphType.UV_Additional3;
                        morph.MorphOffsetes.Add(UVMorphOffset.getUVMorph(fs, header, MorphType.UV_Additional3));
                        break;
                    case 7:
                        morph.type = MorphType.UV_Additional4;
                        morph.MorphOffsetes.Add(UVMorphOffset.getUVMorph(fs, header, MorphType.UV_Additional4));
                        break;
                    case 8:
                        //Material Morph
                        morph.type = MorphType.Matrial;
                        morph.MorphOffsetes.Add(MaterialMorphOffset.getMaterialMorph(fs, header));
                        break;
                    case 9:
                        if (header.Version < 2.1) throw new InvalidDataException("FlipモーフはPMX2.1以降でサポートされています。");
                        morph.type = MorphType.Flip;
                        morph.MorphOffsetes.Add(FlipMorphOffset.getFlipMorph(fs, header));
                        break;
                    case 10:
                        if (header.Version < 2.1) throw new InvalidDataException("ImpulseモーフはPMX2.1以降でサポートされています。");
                        morph.type = MorphType.Impulse;
                        morph.MorphOffsetes.Add(ImpulseMorphOffset.getImpulseMorph(fs, header));
                        break;

                }
            }
            return morph;
        }
        public String MorphName { get; private set; }

        public String MorphName_En { get; private set; }

        public byte OperationPanel { get; private set; }

        public MorphType type { get; private set; }

        public int MorphOffsetCount { get; private set; }

        public List<MorphOffsetBase> MorphOffsetes { get; private set; }
    }

    public enum MorphType
    {
        Vertex=0
            ,UV=1,UV_Additional1=2,UV_Additional2=3,UV_Additional3=4,UV_Additional4=5,Bone=6,Matrial=7,Group=8,Flip=9,Impulse=10
    }
}
