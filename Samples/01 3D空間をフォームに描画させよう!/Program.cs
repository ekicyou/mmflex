using System;
/*
 * *****************************************************************************************************************************************************************
 * MMFチュートリアル 01「3D空間をフォームに描画させよう!」
 * 
 * ◎このセクションの目的
 * 1,MMFのRenderFormを使用する方法を学ぶ
 * 2,MMFのループ方法を学ぶ
 * 
 * ◎所要時間
 * 5分
 * 
 * ◎難易度
 * カンタン♪
 * 
 * ◎前準備
 * ・MMFの参照追加
 * ・SlimDXの参照追加
 * 
 * ◎このチュートリアルの工程
 * ①～②
 * ・Program.cs
 * ・Form1.cs
 * の2ファイルのみ
 * 
 * ◎必須ランタイム
 * DirectX エンドユーザーランタイム
 * SlimDX エンドユーザーランタイム x86 .Net4.0用
 * .Net Framework 4.5
 * 
 * ◎終着点
 * 青い画面が出ればOK
 * 
 ********************************************************************************************************************************************************************/
namespace _01_Initialize3DCGToYourForm
{
    internal static class Program
    {
        /// <summary>
        ///     アプリケーションのメイン エントリ ポイントです。
        /// </summary>
        [STAThread]
        private static void Main()
        {
            //①フォームのメッセージループの方法を変更する。

            //C#でWindowsフォームアプリケーションを作成すると以下のようなプログラムが生成される。
            //Application.EnableVisualStyles();
            //Application.SetCompatibleTextRenderingDefault(false);
            //Application.Run(new Form1());
            //これを以下のように変更する。
            MMF.MessagePump.Run(new Form1());

            /*
             * ◎上の部分の解説
             * 
             * ②を見ればわかるとおり、Form1はRenderFormを継承する。RenderFormはMMFが定義した3DCG空間を描画するフォームの基底クラスとなる。
             * RenderFormは、Renderメソッドを呼び出したときに描画を実行する。つまり、絶えずRenderメソッドを呼び続けることにより3DCG表示が更新されるのである。
             * void MMF.MessagePump.Run(RenderForm form);
             * は、引数に与えられたformのRenderメソッドを呼び続けてくれる静的メソッドである。
             * なお、Alt+Enterでフルスクリーンと切り替えることが可能である。
             */
        }
    }
}