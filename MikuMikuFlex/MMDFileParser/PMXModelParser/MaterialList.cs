using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class MaterialList
    {
        internal static MaterialList getMaterialList(FileStream fs, Header header)
        {
            MaterialList materialList = new MaterialList();
            materialList.Materials = new List<MaterialData>();
            materialList.MaterialCount = ParserHelper.getInt(fs);
            for (int i = 0; i < materialList.MaterialCount; i++)
            {
                materialList.Materials.Add(MaterialData.getMaterial(fs, header));
            }
            return materialList;
        }

        public int MaterialCount { get; private set; }

        public List<MaterialData> Materials { get; private set; }


        public MaterialData getMaterialByIndex(int index)
        {
            int surfaceNum = 0;
            for (int i = 0; i < MaterialCount; i++)
            {
                surfaceNum += Materials[i].VertexNumber / 3;
                if (index < surfaceNum)
                {
                    return Materials[i];
                }
            }
            throw new InvalidDataException();
        }
    }
}
