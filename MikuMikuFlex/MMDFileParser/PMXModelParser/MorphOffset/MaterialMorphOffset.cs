using SlimDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser.MorphOffset
{
    public class MaterialMorphOffset:MorphOffsetBase
    {
        internal static MaterialMorphOffset getMaterialMorph(FileStream fs, Header header)
        {
            MaterialMorphOffset mm = new MaterialMorphOffset();
            mm.type = MorphType.Matrial;
            mm.MaterialIndex = ParserHelper.getIndex(fs, header.MaterialIndexSize);
            mm.OffsetCalclationType = ParserHelper.getByte(fs);
            mm.Diffuse = ParserHelper.getFloat4(fs);
            mm.Specular = ParserHelper.getFloat3(fs);
            mm.SpecularCoefficient = ParserHelper.getFloat(fs);
            mm.Ambient = ParserHelper.getFloat3(fs);
            mm.EdgeColor = ParserHelper.getFloat4(fs);
            mm.EdgeSize = ParserHelper.getFloat(fs);
            mm.TextureCoefficient = ParserHelper.getFloat4(fs);
            mm.SphereTextureCoefficient= ParserHelper.getFloat4(fs);
            mm.ToonTextureCoefficient = ParserHelper.getFloat4(fs);
            return mm;
        }

        public int MaterialIndex { get; private set; }

        public byte OffsetCalclationType { get; private set; }

        public Vector4 Diffuse { get; private set; }

        public Vector3 Specular { get; private set; }

        public float SpecularCoefficient { get; private set; }

        public Vector3 Ambient { get; private set; }

        public Vector4 EdgeColor { get; private set; }

        public float EdgeSize { get; private set; }

        public Vector4 TextureCoefficient { get; private set; }

        public Vector4 SphereTextureCoefficient { get; private set; }

        public Vector4 ToonTextureCoefficient { get; private set; }
    }
}
