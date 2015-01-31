using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;

namespace MMF.Utility
{
    /// <summary>
    /// tgaファイルをDX11では読み込めないので、一度pngに変換してから読み込む
    /// </summary>
    public static class TargaSolver
    {
        public static Stream LoadTargaImage(string filePath,ImageFormat rootFormat=null)
        {
            Bitmap tgaFile = null;
            if (rootFormat == null) rootFormat = ImageFormat.Png;
            try
            {
                tgaFile = Paloma.TargaImage.LoadTargaImage(filePath);
            }
            catch (Exception ns)
            {//tga以外の形式のとき
                return File.OpenRead(filePath);
            }
            MemoryStream ms=new MemoryStream();
                tgaFile.Save(ms,rootFormat);
                ms.Seek(0, SeekOrigin.Begin);
                return ms;
        }
    }
}
