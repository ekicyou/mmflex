using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

// 自動実装コード VmeProto の追加要素
namespace OpenMMDFormat
{
    /// <summary>
    /// BoneFrameクラスにIFrameDataインターフェースを追加実装
    /// </summary>
    public partial class BoneFrame : MMDFileParser.IFrameData
    {
        public uint FrameNumber
        {
            get { return (uint)_frameNumber; }
        }

        public int CompareTo(Object x)
        {
            return (int)FrameNumber - (int)((MMDFileParser.IFrameData)x).FrameNumber;
        }
    }

    /// <summary>
    /// MorphFrameクラスにIFrameDataインターフェースを追加実装
    /// </summary>
    public partial class MorphFrame : MMDFileParser.IFrameData
    {
        public uint FrameNumber
        {
            get { return (uint)_frameNumber; }
        }

        public int CompareTo(Object x)
        {
            return (int)FrameNumber - (int)((MMDFileParser.IFrameData)x).FrameNumber;
        }
    }
}
