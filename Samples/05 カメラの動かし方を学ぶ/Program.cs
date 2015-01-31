using System;
using System.Windows.Forms;
/*
 * *****************************************************************************************************************************************************************
 * MMFチュートリアル 05「カメラの動かし方を学ぶ」
 * 
 * ◎このセクションの目的
 * 1,MMFでどのようにカメラを動かすか学ぶ
 * 
 * ◎所要時間
 * 20分
 * 
 * ◎難易度
 * カンタン♪
 * 
 * ◎前準備
 * ・03までの内容を作成したプロジェクトを用意する
 * ★★★重要★★★→Sampleの中のMMFLib内に含まれるx86,x64,Toon,Shaderフォルダーを出力先のフォルダにコピーする。bin\\Debug内など。
 * 
 * ◎このチュートリアルの工程
 * ①～③
 * ・Form1.cs
 * ・CameraControlSelector.cs
 * ・SimpleUserDefinitionCameraMotionProvider.cs
 * の1ファイルのみ
 * 
 * ◎必須ランタイム
 * DirectX エンドユーザーランタイム
 * SlimDX エンドユーザーランタイム x86 .Net4.0用
 * .Net Framework 4.5
 * 
 * ◎終着点
 * カメラが動けばOK
 * 
 ********************************************************************************************************************************************************************/
using MMF;

namespace _05_HowToUpdateCamera
{
    static class Program
    {
        /// <summary>
        /// アプリケーションのメイン エントリ ポイントです。
        /// </summary>
        [STAThread]
        static void Main()
        {
            MessagePump.Run(new Form1());
        }
    }
}
