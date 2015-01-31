using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using CGTest.Properties;
using MMF;
using SlimDX.Windows;
using MessagePump = MMF.MessagePump;

namespace CGTest
{
    static class Program
    {
        /// <summary>
        /// アプリケーションのメイン エントリ ポイントです。
        /// 
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.SetUnhandledExceptionMode(UnhandledExceptionMode.CatchException);
            Application.ThreadException += Application_ThreadException;
            Thread.GetDomain().UnhandledException += Application_UnhandledException;
            Application.EnableVisualStyles(); 
            Application.SetCompatibleTextRenderingDefault(false);
            Form1 form=new Form1();
            MessagePump.Run(form);
            Settings.Default.Save();
            /*
             * MMDDを利用する場合２つの方法が存在する。
             * ①[編集ソフトなどを作成する場合にはお勧め。]RenderPanel系ユーザーコントロールを貼り付ける。(MMDD.CG.RenderPanelもしくは、MMDD.CG.D2DSupportedRenderPanelの場合)
             * この場合、自分で用意したフォームに対してユーザーコントロールを貼り付け、OnLoadメソッドなどをオーバーライドしInitializeメソッドを呼ぶ。
             * ユーザーコントロール上に描画されるため、同じフォームに対してボタンなどを配置できたり、複数RenderPanelを配置することが可能。
             * 欠点として、フルスクリーンにできないこと、②と比べると描画速度が低下することがある。
             * 
             * RenderPanel系のコントロールを継承する場合、以下のことに気をつけること。
             * RenderPanel内のOnPaintでRenderを呼び出し、Invalidateを呼び出す ことによるループは一般的に通じない
             * ちらつきを引き起こすので、RenderはForm側のOnPaintに同期して呼び出されるのが望ましい。
             * 
             * ②[ゲームなどを作成する場合にはおすすめ。]RenderForm系フォームを継承する。(MMDD.RenderFormもしくは、MMDD.D2DSupportedRenderFormの場合)
             * この場合、自分で用意したフォームには継承して適用する。Renderメソッドを呼び出せば、描画されるがこの描画用ループには2種類の方法がサポートされている。
             * (A)[推奨]MMDD.MessagePumpクラス、Runメソッドを利用する場合
             * MMDD.MessagePump(フォームのインスタンス);によってループをする。
             * ただし、フォームの閉じるボタンは動作しない。ゲーム内で終了ボタンなどを用意して、Closeメソッドを呼ぶ必要があります。
             * (B)RenderForm.DoOnPaintLoopをtrueとする方法。
             * RenderForm.DoOnPaintLoop=trueのときは、描画をPaintループで行われるように実装されている。Paint内でInvalidateを呼び出すことで再描画を通知するが、メモリを浪費するため
             * 使用は推薦されない。ただし、閉じるボタンを動作する。
             *
             * ループを指定する方法がどちらであっても、フルスクリーンは動作する。RenderForm.RenderContext.DeviceManager.SwapChain.IsFullScreen=trueとすればよい。
             * 
             * For using MMDD,we provide two ways below.
             * ①[We recommend this way for the app kind of editor]
             * Attach MMDD.CG.RenderPanel user-control to your apps Form.(MMDD.CG.RenderPanel or MMDD.CG.D2DSupportedRenderPanel)
             * In this way,you should call RenderPanel.Initialize() method in your Form initializing stage like overriding OnLoad.
             * Any MMDD's 3DCG will be rendering on UserControl. So,you can put Button and some of RenderPanel on same Form.
             * As week point,you can't turn on fullscreen in this mode,and this mode rendering speed is slower than ②.
             * You should call Render method when you want to redraw control. In general,you call Render method in the Form's OnPaint.
             * 
             * When you extends RenderPanel,remember not to do as below.
             * Override RenderPanel's OnPaint,call Invalidate after calling Render.
             * 
             * If you write like that,it cause your flickering to your application.
             * You should call Render method when parent form is painted.
             * 
             * ②[We recommend this way for the app kind of games]
             * To use MMDD,your apps Form extends the class kind of RenderForm[MMDD.RenderForm or MMDD.D2DSupportedRenderForm]
             * In this way,you should consider how to call RenderForm.Render() method frequently.
             * We provides two ways below.
             * (A)[Recommended]Using static method MMDD.MessagePump.Run(RenderForm form)
             * You should use MMDD.MessagePump.Run method insted of Application.Run method in your Program.cs usually used as entry point.
             * This way can be most efficient way to loop method. As week point,Form's close button will not work. You should call Close method to close Form.
             * (B)the way to make RenderForm.DoOnPaintLoop=true
             * When RenderForm.DoOnPaintLoop==true,RenderForm will loop Render method with OnPaint override.
             * This is the easist way to loop Render method. But,this way can use a large amount of memory.
             * Form's close button will work.
             * 
             * Both (A) and (B) can use fullscreen.
             */
        }

        private static void Application_UnhandledException(object sender, UnhandledExceptionEventArgs e)
        {
            StreamWriter writer = new StreamWriter("log.txt");
            writer.Write(e.ExceptionObject.ToString());
            writer.Flush();
            writer.Close();
        }

        static void Application_ThreadException(object sender, System.Threading.ThreadExceptionEventArgs e)
        {
            StreamWriter writer=new StreamWriter("log.txt");
            writer.Write(e.Exception.ToString());
            writer.Flush();
            writer.Close();
        }
    }
}
