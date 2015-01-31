using System.Diagnostics;
using System.IO;
using MMF.Utility;

namespace MMF.Model
{
    /// <summary>
    ///     標準的なリソース読み込み、指定したディレクトリをベースとしてリソースを読み込む
    /// </summary>
    public class BasicSubresourceLoader : ISubresourceLoader
    {
        /// <summary>
        ///     コンストラクタ
        /// </summary>
        /// <param name="baseDir">ベースとするディレクトリ</param>
        public BasicSubresourceLoader(string baseDir)
        {
            if (string.IsNullOrEmpty(baseDir))
            {
                BaseDirectory = ".\\";
                return;
            }
            BaseDirectory = Path.GetFullPath(baseDir);
        }

        /// <summary>
        ///     ベースディレクトリ
        /// </summary>
        public string BaseDirectory { get; set; }

        /// <summary>
        ///     指定したリソースを読み込む
        /// </summary>
        /// <param name="name">リソース名</param>
        /// <returns>リソースのストリーム</returns>
        public Stream getSubresourceByName(string name)
        {
            if (Path.GetExtension(name).ToUpper().Equals(".TGA"))
            {
                if (string.IsNullOrEmpty(BaseDirectory)) return TargaSolver.LoadTargaImage(name);
                return TargaSolver.LoadTargaImage(Path.Combine(BaseDirectory, name));
            }else if (string.IsNullOrEmpty(BaseDirectory)) return File.OpenRead(name);
            else
            {
                string path = Path.Combine(BaseDirectory, name);
                if (File.Exists(path))
                {
                    return File.OpenRead(path);
                }
                else
                {
                    return null;
                     Debug.WriteLine(string.Format("\"{0}\"は見つかりませんでした。",path));
                }
            }
        }
    }
}