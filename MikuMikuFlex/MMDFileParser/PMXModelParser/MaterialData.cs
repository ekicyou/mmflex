using SlimDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class MaterialData
    {
        internal static MaterialData getMaterial(FileStream fs, Header header)
        {
            MaterialData material = new MaterialData();
            material.MatrialName = ParserHelper.getTextBuf(fs, header.Encode);
            material.MaterialName_En = ParserHelper.getTextBuf(fs, header.Encode);
            material.Diffuse = ParserHelper.getFloat4(fs);
            material.Specular = ParserHelper.getFloat3(fs);
            material.SpecularCoefficient = ParserHelper.getFloat(fs);
            material.Ambient = ParserHelper.getFloat3(fs);
            material.bitFlag = (RenderFlag)ParserHelper.getByte(fs);
            material.EdgeColor = ParserHelper.getFloat4(fs);
            material.EdgeSize = ParserHelper.getFloat(fs);
            material.TextureTableReferenceIndex = ParserHelper.getIndex(fs, header.TextureIndexSize);
            material.SphereTextureTableReferenceIndex = ParserHelper.getIndex(fs, header.TextureIndexSize);
            switch (ParserHelper.getByte(fs))
            {
                case 0:
                    material.SphereMode = SphereMode.Disable;
                    break;
                case 1:
                    material.SphereMode = SphereMode.Multiply;
                    break;
                case 2:
                    material.SphereMode = SphereMode.Add;
                    break;
                case 3:
                    material.SphereMode = SphereMode.SubTexture;
                    break;
                default:
                    throw new InvalidDataException("スフィアモード値が以上です");
            }
            material.ShareToonFlag = ParserHelper.getByte(fs);
            material.textureIndex = material.ShareToonFlag == 0 ? ParserHelper.getIndex(fs, header.TextureIndexSize) : ParserHelper.getByte(fs);
            material.Memo = ParserHelper.getTextBuf(fs, header.Encode);
            material.VertexNumber = ParserHelper.getInt(fs);
            if (material.VertexNumber % 3 != 0) throw new InvalidDataException();
            return material;
        }

        public String MatrialName;

        public String MaterialName_En;

        public Vector4 Diffuse;

        public Vector3 Specular;

        public float SpecularCoefficient;

        public Vector3 Ambient;

        public RenderFlag bitFlag;

        public Vector4 EdgeColor;

        public float EdgeSize;

        public int TextureTableReferenceIndex;

        public int SphereTextureTableReferenceIndex;

        public SphereMode SphereMode;

        public byte ShareToonFlag;
        /// <summary>
        /// ShareToonFlagが0の時は、Toonテクスチャテクスチャテーブルの参照Index
        /// ShareToonFlagが1の時は、共有Toonテクスチャ[0~9]→それぞれtoon01.bmp~toon10.bmpに対応
        /// </summary>
        public int textureIndex;


        public String Memo;

        public int VertexNumber;
    }
}
