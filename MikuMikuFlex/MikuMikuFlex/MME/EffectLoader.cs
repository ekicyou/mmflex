using System;
using System.Data;
using System.Data.SQLite;
using System.Diagnostics;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using SlimDX;
using SlimDX.D3DCompiler;
using SlimDX.XAudio2;
using Application = System.Windows.Forms.Application;

namespace MMF.MME
{
    /// <summary>
    ///     エフェクトのキャッシュをとっておくクラス
    ///     コンパイル回数を少なくするために効率的な初期化が可能
    /// </summary>
    public class EffectLoader : IDisposable
    {
        private static EffectLoader instance;

        private static readonly string cacheFileName = "effect.cache";

        private static readonly string connectionString = "Data Source=effect.cache";

        private static readonly string tableCreationSQL
            = @"CREATE TABLE 'DBHeader' (
             'Id' INTEGER PRIMARY KEY ON CONFLICT FAIL AUTOINCREMENT UNIQUE ON CONFLICT FAIL DEFAULT '',
            'Property' CHAR NOT NULL ON CONFLICT FAIL,
            'Value' CHAR);
            INSERT INTO DBHeader VALUES(NULL,'DBType','EffectCacheDatabase');
            INSERT INTO DBHeader VALUES(NULL,'FileVersion','1.0');
            CREATE TABLE 'EffectCache'(
            'Id' INTEGER PRIMARY KEY ON CONFLICT FAIL AUTOINCREMENT UNIQUE ON CONFLICT FAIL DEFAULT '',
            'FileName' CHAR NOT NULL ON CONFLICT FAIL,
            'HashCode' CHAR NOT NULL ON CONFLICT FAIL,
            'ShaderByteCode' BLOB);";

        private static readonly string getBlobQuery
            = "SELECT ShaderByteCode FROM EffectCache WHERE FileName=='{0}' AND HashCode=='{1}';";

        private static readonly string getByFileNameQuery
            = "SELECT Id FROM EffectCache WHERE FileName=='{0}';";

        private static readonly string deleteByIdQuery
            = "DELETE FROM EffectCache WHERE Id=={0};";

        private static readonly string insertSQL
            = "INSERT INTO EffectCache VALUES(NULL,'{0}','{1}',@resource);";

        public EffectLoader()
        {
            bool isExist = File.Exists(cacheFileName);
            Connection = new SQLiteConnection(connectionString);
            Connection.Open();
            if (!isExist)
            {
                using (SQLiteCommand command = new SQLiteCommand(tableCreationSQL, Connection))
                {
                    command.ExecuteNonQuery();
                }
            }
        }


        public static EffectLoader Instance
        {
            get
            {
                if (instance == null) instance = new EffectLoader();
                return instance;
            }
            private set { instance = value; }
        }

        public event EventHandler<EffectLoaderCompilingEventArgs> OnCompiling = delegate { };

        public event EventHandler<EffectLoaderCompiledEventArgs> OnCompiled = delegate { }; 

        public SQLiteConnection Connection { get; set; }

        /// <summary>
        ///     キャッシュをクリアします
        /// </summary>
        public void CleanCache()
        {
            bool isExist = File.Exists(cacheFileName);
            if (isExist)
            {
                File.Delete(cacheFileName);
            }
        }

        /// <summary>
        ///     指定したファイル名から、エフェクトを取得します。
        /// </summary>
        /// <param name="fileStream">エフェクトのファイル名</param>
        /// <returns>コンパイル済みエフェクト</returns>
        public ShaderBytecode GetShaderBytecode(string fileName, Stream fileStream)
        {
            string hashStr = getFileHash(fileStream); //指定したファイルのハッシュを取得
            fileStream.Seek(0, SeekOrigin.Begin);
            using (
                SQLiteCommand getBlobCommand = new SQLiteCommand(string.Format(getBlobQuery, fileName, hashStr),
                    Connection))
            using (SQLiteDataReader blobReader = getBlobCommand.ExecuteReader())
            {
                if (blobReader.Read()) //ハッシュとファイル名が等しい場合(ファイルが更新されず、名前も等しい時)はデータベースから取得
                {
                    MemoryStream ms = getBytesToMemoryStream(blobReader, 0);
                    DataStream ds = new DataStream(ms.Length, true, true);
                    ms.CopyTo(ds);
                    return new ShaderBytecode(ds);
                } 
                else
                {
                    StreamReader reader = new StreamReader(fileStream);
                    string shaderCode = reader.ReadToEnd();
                    //キャッシュから取得できなかった場合はコンパイルが必要と判断しコンパイルする
                    ShaderFlags flags=ShaderFlags.None;
#if DEBUG
                    flags|=ShaderFlags.Debug;
#endif
                    Debug.WriteLine("Start compiling shader:{0},please wait...",fileName);
                    OnCompiling(this,new EffectLoaderCompilingEventArgs(fileName));
                    ShaderBytecode sbc = null;
                    Task t= new Task(() =>
                    {
                         sbc= ShaderBytecode.Compile(shaderCode, "fx_5_0", ShaderFlags.Debug,
                            EffectFlags.None, MMEEffectManager.EffectMacros.ToArray(), MMEEffectManager.EffectInclude);
                        using (
                            SQLiteCommand getFileByNameCommand =
                                new SQLiteCommand(string.Format(getByFileNameQuery, fileStream), Connection))
                        using (SQLiteDataReader fileNameReader = getFileByNameCommand.ExecuteReader())
                            //コンパイルできた場合は、同じファイル名で別のハッシュ値は古いファイルと判断しキャッシュから削除する
                        {
                            while (fileNameReader.Read())
                            {
                                using (
                                    SQLiteCommand deleteSameNameCommand =
                                        new SQLiteCommand(string.Format(deleteByIdQuery, fileNameReader.GetInt32(0)),
                                            Connection))
                                {
                                    deleteSameNameCommand.ExecuteNonQuery();
                                }
                            }
                        }
                        //新たなキャッシュとしてエフェクトを記録する
                        MemoryStream ms = new MemoryStream();
                        sbc.Data.CopyTo(ms);
                        byte[] sbcBytes = ms.ToArray();
                        using (
                            SQLiteCommand blobInsertCommand = new SQLiteCommand(
                                string.Format(insertSQL, fileName, hashStr), Connection))
                        {
                            blobInsertCommand.Parameters.Add("@resource", DbType.Binary, sbcBytes.Length).Value =
                                sbcBytes;
                            blobInsertCommand.ExecuteNonQuery();
                        }
                    });
                    t.Start();
                    while (!t.IsCompleted)
                    {
                        Application.DoEvents();
                    }
                    OnCompiled(this,new EffectLoaderCompiledEventArgs());
                    return sbc;
                }
            }
        }

        protected MemoryStream getBytesToMemoryStream(SQLiteDataReader reader, int index)
        {
            MemoryStream ms = new MemoryStream();
            byte[] buffer = new byte[2048];
            long readed;
            long offset = 0;
            while ((readed = reader.GetBytes(index, offset, buffer, 0, buffer.Length)) > 0)
            {
                ms.Write(buffer, 0, (int) readed);
                offset += readed;
            }
            ms.Seek(0, SeekOrigin.Begin);
            return ms;
        }

        /// <summary>
        ///     ファイルパスからハッシュ値を取得する
        /// </summary>
        /// <param name="filePath"></param>
        /// <returns></returns>
        private string getFileHash(string filePath)
        {
            using (FileStream fs = File.OpenRead(filePath))
                return getFileHash(fs);
        }

        /// <summary>
        ///     ファイルストリームからハッシュ値を取得
        /// </summary>
        /// <param name="stream"></param>
        /// <returns></returns>
        private string getFileHash(Stream stream)
        {
            using (MD5 md5 = MD5.Create())
            {
                byte[] hashBytes = md5.ComputeHash(stream);
                StringBuilder builder = new StringBuilder();
                foreach (byte hashByte in hashBytes)
                {
                    builder.Append(hashByte.ToString("x2"));
                }
                return builder.ToString();
            }
        }

        #region IDisposable メンバー

        public void Dispose()
        {
            if (Connection != null) Connection.Close();
        }

        #endregion
    }

    public class EffectLoaderCompilingEventArgs : EventArgs
    {
        private string targetFileName;

        public string TargetFileName
        {
            get { return targetFileName; }
        }

        public EffectLoaderCompilingEventArgs(string targetFileName)
        {
            this.targetFileName = targetFileName;
        }
    }

    public class EffectLoaderCompiledEventArgs : EventArgs
    {
        
    }
}