namespace MMF.MME.Includer
{
    /// <summary>
    ///     Includeファイルの検索に利用するディレクトリ
    /// </summary>
    public class IncludeDirectory
    {
        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="directory">ディレクトリパス</param>
        /// <param name="priorty">優先度</param>
        public IncludeDirectory(string directory, int priorty)
        {
            DirectoryPath = directory;
            Priorty = priorty;
        }

        /// <summary>
        /// ディレクトリのパス
        /// </summary>
        public string DirectoryPath { get; private set; }

        /// <summary>
        /// ディレクトリの優先度
        /// </summary>
        public int Priorty { get; private set; }
    }
}