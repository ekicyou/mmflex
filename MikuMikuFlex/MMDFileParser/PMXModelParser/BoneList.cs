﻿using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    public class BoneList
    {
        internal static BoneList getBoneList(FileStream fs, Header header)
        {
            BoneList boneList = new BoneList();
            boneList.Bones = new List<BoneData>();
            boneList.BoneCount = ParserHelper.getInt(fs);
            for (int i = 0; i < boneList.BoneCount; i++)
            {
                boneList.Bones.Add(BoneData.getBone(fs, header));
            }
            return boneList;
        }

        public int BoneCount{get;private set;}

        public List<BoneData> Bones { get; private set; }
    }
}
