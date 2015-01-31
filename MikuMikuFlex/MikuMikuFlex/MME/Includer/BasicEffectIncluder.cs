using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;
using SlimDX.D3DCompiler;

namespace MMF.MME.Includer
{
    /// <summary>
    /// 基本的な#includeのパス解決クラス
    /// </summary>
    public class BasicEffectIncluder
        : Include,IComparer<IncludeDirectory>

    {
        /// <summary>
        /// コンストラクタ
        /// </summary>
        public BasicEffectIncluder()
        {
            IncludeDirectories = new ObservableCollection<IncludeDirectory>();
            IncludeDirectories.CollectionChanged += IncludeDirectories_CollectionChanged;
            IncludeDirectories.Add(new IncludeDirectory("Shader\\include",0));
        }

        /// <summary>
        /// コレクションが変更されたときはソートし直す
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        void IncludeDirectories_CollectionChanged(object sender, System.Collections.Specialized.NotifyCollectionChangedEventArgs e)
        {
            List<IncludeDirectory> sorted = IncludeDirectories.ToList();
            IncludeDirectories=new ObservableCollection<IncludeDirectory>(sorted);
        }

        /// <summary>
        /// 登録されているディレクトリとその優先度のリスト
        /// </summary>
        public ObservableCollection<IncludeDirectory> IncludeDirectories { get; private set; }

        #region Include メンバー

        public void Close(Stream stream)
        {
            stream.Close();
        }

        /// <summary>
        /// ファイルを開くときに利用する
        /// </summary>
        /// <param name="type"></param>
        /// <param name="fileName"></param>
        /// <param name="parentStream"></param>
        /// <param name="stream"></param>
        public void Open(IncludeType type, string fileName, Stream parentStream, out Stream stream)
        {
            if (Path.IsPathRooted(fileName))//絶対パスならそのまま帰す
            {
                stream = File.OpenRead(fileName);
                return;
            }
            foreach (IncludeDirectory directory in IncludeDirectories)
            {
                if (File.Exists(Path.Combine(directory.DirectoryPath, fileName)))
                {
                    stream = File.OpenRead(Path.Combine(directory.DirectoryPath, fileName));
                    return;
                }
            }
            stream = null;
        }

        #endregion


        #region IComparer<IncludeDirectory> メンバー

        /// <summary>
        /// 比較するときに利用するインターフェース
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        public int Compare(IncludeDirectory x, IncludeDirectory y)
        {
            return x.Priorty - y.Priorty;
        }

        #endregion
    }
}