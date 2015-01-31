using MMDFileParser.PMXModelParser;
using SlimDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SlimDX.Direct3D10;

namespace MMDFileParser
{
    internal static class ParserHelper
    {
        internal static string getTextBuf(Stream fs,EncodeType encode)
        {
            byte[] strLength = new byte[4];
            fs.Read(strLength, 0, 4);
            int Length = BitConverter.ToInt32(strLength, 0);
            byte[] StrBuf=new byte[Length];
            fs.Read(StrBuf,0,Length);
            if (encode == EncodeType.UTF8)
            {
                return Encoding.UTF8.GetString(StrBuf);
            }
            else
            {
                return Encoding.Unicode.GetString(StrBuf);
            }
        }
        internal static float getFloat(Stream fs)
        {
            byte[] buffer = new byte[4];
            fs.Read(buffer, 0, 4);
            return BitConverter.ToSingle(buffer, 0);
        }
        internal static Vector4 getFloat4(Stream fs)
        {
            byte[] buffer = new byte[16];
            fs.Read(buffer, 0, 16);
            return new Vector4(BitConverter.ToSingle(buffer, 0), BitConverter.ToSingle(buffer, 4),BitConverter.ToSingle(buffer,8),BitConverter.ToSingle(buffer,12));
        }

        internal static Vector3 getFloat3(Stream fs)
        {
            byte[] buffer = new byte[12];
            fs.Read(buffer, 0, 12);
            return new Vector3(BitConverter.ToSingle(buffer,0),BitConverter.ToSingle(buffer,4),BitConverter.ToSingle(buffer,8));
        }

        internal static Vector2 getFloat2(Stream fs)
        {
            byte[] buffer = new byte[8];
            fs.Read(buffer, 0, 8);
            return new Vector2(BitConverter.ToSingle(buffer,0),BitConverter.ToSingle(buffer,4));
        }

        internal static int getInt(Stream fs)
        {
            byte[] buffer = new byte[4];
            fs.Read(buffer, 0, 4);
            return BitConverter.ToInt32(buffer, 0);
        }

        internal static ushort getUShort(Stream fs)
        {
            byte[] buffer = new byte[2];
            fs.Read(buffer, 0, 2);
            return BitConverter.ToUInt16(buffer,0);
        }

        internal static byte getByte(Stream fs)
        {
            byte[] buffer = new byte[1];
            fs.Read(buffer, 0, 1);
            return buffer[0];
        }


        internal static int getIndex(Stream fs,int size)
        {
            byte[] buffer = new byte[size];
            fs.Read(buffer, 0, size);
            switch (size)
            {
                case 1:
                    return (sbyte)buffer[0];
                case 2:
                    return BitConverter.ToInt16(buffer, 0);
                case 4:
                    return BitConverter.ToInt32(buffer, 0);
                default:
                    throw new InvalidDataException();
            }
        }

        internal static uint getVertexIndex(Stream fs, int size)
        {
            byte[] buffer = new byte[size];
            fs.Read(buffer, 0, size);
            switch (size)
            {
                case 1:
                    return buffer[0];
                case 2:
                    return BitConverter.ToUInt16(buffer, 0);
                case 4:
                    return BitConverter.ToUInt32(buffer, 0);
                default:
                    throw new InvalidDataException();
            }
        }

        internal static bool isFlagEnabled(short chk, short flag)
        {
            return (chk & flag) == flag;
        }

        internal static String getShift_JISString(Stream fs,int length)
        {
            Encoding en = Encoding.GetEncoding("Shift_JIS");
            List<byte> textBuf = new List<byte>();
            for (int i = 0; i < length; i++)
            {
                byte[] t =new byte[1]{ getByte(fs)};
                if (en.GetString(t)[0] == '\0')
                {
                    fs.Read(new byte[length-(i+1)], 0, length - (i + 1));
                    break;
                }
                else
                {
                    textBuf.Add(t[0]);
                }
            }
            return en.GetString(textBuf.ToArray());
        }

        internal static uint getDWORD(Stream fs)
        {
            byte[] buffer = new byte[4];
            if (fs.Read(buffer, 0, 4) == 0) throw new EndOfStreamException();
            return BitConverter.ToUInt32(buffer, 0);
        }

        internal static Quaternion getQuaternion(Stream fs)
        {
            byte[] buffer = new byte[16];
            fs.Read(buffer, 0, 16);
            return new Quaternion(BitConverter.ToSingle(buffer, 0), BitConverter.ToSingle(buffer, 4), BitConverter.ToSingle(buffer, 8), BitConverter.ToSingle(buffer, 12));
        }
    }
}
