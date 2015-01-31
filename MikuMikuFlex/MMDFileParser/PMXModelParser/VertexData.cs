using SlimDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    /// <summary>
    /// 頂点データの格納クラス
    /// </summary>
    public class VertexData
    {
        internal static VertexData getVertex(FileStream fs, Header header)
        {
            VertexData vertex = new VertexData();
            vertex.Position = ParserHelper.getFloat3(fs);
            vertex.Normal = ParserHelper.getFloat3(fs);
            vertex.UV = ParserHelper.getFloat2(fs);
            vertex.AdditionalUV = new Vector4[header.AdditionalUVCount];
            for (int i = 0; i < header.AdditionalUVCount; i++)
            {
                vertex.AdditionalUV[i] = ParserHelper.getFloat4(fs);
            }
            switch (ParserHelper.getByte(fs))
            {
                //BDEF1
                case 0:
                    vertex.TranslationType = WeightTranslationType.BDEF1;
                    vertex.BoneWeight = new BoneWeight.BDEF1() { boneIndex = ParserHelper.getIndex(fs, header.BoneIndexSize) };
                    break;
                case 1:
                    vertex.TranslationType = WeightTranslationType.BDEF2;
                    vertex.BoneWeight = new BoneWeight.BDEF2() { Bone1ReferenceIndex = ParserHelper.getIndex(fs, header.BoneIndexSize), Bone2ReferenceIndex = ParserHelper.getIndex(fs, header.BoneIndexSize), Weight = ParserHelper.getFloat(fs) };
                    break;
                case 2:
                    vertex.TranslationType = WeightTranslationType.BDEF4;
                    vertex.BoneWeight = new BoneWeight.BDEF4() { Bone1ReferenceIndex = ParserHelper.getIndex(fs, header.BoneIndexSize), Bone2ReferenceIndex = ParserHelper.getIndex(fs, header.BoneIndexSize), Bone3ReferenceIndex = ParserHelper.getIndex(fs, header.BoneIndexSize), Bone4ReferenceIndex = ParserHelper.getIndex(fs, header.BoneIndexSize), Weights = ParserHelper.getFloat4(fs) };
                    break;
                case 3:
                    vertex.TranslationType = WeightTranslationType.SDEF;
                    vertex.BoneWeight = new BoneWeight.SDEF() {Bone1ReferenceIndex=ParserHelper.getIndex(fs,header.BoneIndexSize),Bone2ReferenceIndex=ParserHelper.getIndex(fs,header.BoneIndexSize),Bone1Weight=ParserHelper.getFloat(fs),SDEF_C=ParserHelper.getFloat3(fs),SDEF_R0=ParserHelper.getFloat3(fs),SDEF_R1=ParserHelper.getFloat3(fs) };
                    break;
                case 4:
                    if (header.Version != 2.1f) throw new InvalidDataException("QDEFはPMX2.1でのみサポートされます。");
                    vertex.TranslationType = WeightTranslationType.QDEF;
                    vertex.BoneWeight = new BoneWeight.QDEF() { Bone1ReferenceIndex = ParserHelper.getIndex(fs, header.BoneIndexSize), Bone2ReferenceIndex = ParserHelper.getIndex(fs, header.BoneIndexSize), Bone3ReferenceIndex = ParserHelper.getIndex(fs, header.BoneIndexSize), Bone4ReferenceIndex = ParserHelper.getIndex(fs, header.BoneIndexSize), Weights = ParserHelper.getFloat4(fs) };
                    break;
                default:
                    throw new InvalidDataException();
            }
            vertex.EdgeMagnification = ParserHelper.getFloat(fs);
            return vertex;
        }
        public Vector3 Position ;

        public Vector3 Normal ;

        public Vector2 UV ;

        public Vector4[] AdditionalUV ;

        public WeightTranslationType TranslationType ;

        public BoneWeight.BoneWeightBase BoneWeight ;

        public float EdgeMagnification;
    }

    public enum WeightTranslationType
    {
        BDEF1,BDEF2,BDEF4,SDEF,QDEF
    }
}
