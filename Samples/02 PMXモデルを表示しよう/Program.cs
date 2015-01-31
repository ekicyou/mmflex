using System;
using System.Windows.Forms;
using MMF;

/*
 * *****************************************************************************************************************************************************************
 * MMFチュートリアル 02「PMXモデルを表示させよう!」
 * 
 * ◎このセクションの目的
 * 1,MMFでどのようにPMXモデルを表示するか学ぶ
 * 
 * ◎所要時間
 * 5分
 * 
 * ◎難易度
 * カンタン♪
 * 
 * ◎前準備
 * ・01までの内容を作成したプロジェクトを用意する
 * ★★★重要★★★→Sampleの中のMMFLib内に含まれるx86,x64,Toon,Shaderフォルダーを出力先のフォルダにコピーする。bin\\Debug内など。
 * 
 * ◎このチュートリアルの工程
 * ①～③
 * ・Form1.cs
 * の1ファイルのみ
 * 
 * ◎必須ランタイム
 * DirectX エンドユーザーランタイム
 * SlimDX エンドユーザーランタイム x86 .Net4.0用
 * .Net Framework 4.5
 * 
 * ◎終着点
 * モデルが出ればOK
 * 
 ********************************************************************************************************************************************************************/

namespace _02_SimpleRenderPMX
{
    internal static class Program
    {
        /// <summary>
        ///     アプリケーションのメイン エントリ ポイントです。
        /// </summary>
        [STAThread]
        private static void Main()
        {
            Application.ThreadException += Application_ThreadException;
            MessagePump.Run(new Form1());
        }

        static void Application_ThreadException(object sender, System.Threading.ThreadExceptionEventArgs e)
        {
            
        }
    }
}