using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.PMXModelParser
{
    /// <summary>
    /// PMX仕様521行目参照、枠内要素にあたるクラス
    /// </summary>
    public class FrameElementData
    {
        internal static FrameElementData GetFrameElementData(Stream fs,Header header)
        {
            FrameElementData data=new FrameElementData();
            data.IsMorph = ParserHelper.getByte(fs) == 1;
            if (data.IsMorph)
            {
                data.Index = ParserHelper.getIndex(fs, header.MorphIndexSize);
            }
            else
            {
                data.Index = ParserHelper.getIndex(fs, header.BoneIndexSize);
            }
            return data;
        }

        /// <summary>
        /// この枠がモーフかどうか
        /// falseならば要素対象はボーンのインデックス
        /// </summary>
        public bool IsMorph { get; private set; }

        /// <summary>
        /// この要素対象のインデックス
        /// </summary>
        public int Index { get; private set; }
    }
}
