using System;
using System.Drawing;
using System.Windows.Forms;
using MMF;
using MMF.Controls.Forms;
using MMF.Model.PMX;
using MMF.Sprite.D2D;
using SlimDX.Direct2D;
using SlimDX.DirectWrite;
using FontWeight = SlimDX.DirectWrite.FontWeight;

namespace _07_Drawing2DToRenderForm
{
    //①2D表示を利用するため、RenderFormを継承したクラスD2DSupportedRenderFormを利用するとよい。
    public partial class Form1 : D2DSupportedRenderForm
    {
        public Form1()
        {
            InitializeComponent();
        }
        #region ②ブラシ、テキストフォーマットなどの初期化

        private D2DSpriteTextformat textFormat;

        private D2DSpriteSolidColorBrush colorBrush;

        private D2DSpriteBitmap bitmap;

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            //②-A 描画に利用するリソースの初期化
            textFormat = SpriteBatch.CreateTextformat("Meiriyo", 30, FontWeight.Bold);
            //テキストフォーマットの揃えの指定は以下のように行う。
            textFormat.Format.ParagraphAlignment = ParagraphAlignment.Center; //縦方向の揃え。Center:中央、Near:上、Far:下
            textFormat.Format.TextAlignment = TextAlignment.Center; //横方向の揃え。Center:中央、Near:左、Far:右


            colorBrush = SpriteBatch.CreateSolidColorBrush(Color.OrangeRed);


            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Filter = "ビットマップファイル(*.bmp)|*.bmp|PNGファイル(*.png)|*.png|JPGファイル(*.jpg)|*.jpg|すべてのファイル(*.*)|*.*";
            if (ofd.ShowDialog(this)==DialogResult.OK)
            {
                bitmap = SpriteBatch.CreateBitmap(ofd.FileName);
            }else
                Close(); 

            
        

        /*
             * 画面がリサイズされた時などには、SpriteBatchが自動的にリサイズされる。
             * この時、リソースも作り直す必要性があるためSpriteBatch.Create~~として取得できるD2DSprite~~は自動的に
             * リサイズ時などに同じ設定で再作成される。
             * ただし、すべてのDirectWriteに利用するクラスについてこれは実装を完了していない。
             */
            OpenFileDialog ofd2=new OpenFileDialog();
            ofd2.Filter = "PMXモデルファイル(*.pmx)|*.pmx";
            if (ofd2.ShowDialog(this) == DialogResult.OK)
            {
                WorldSpace.AddResource(PMXModelWithPhysics.OpenLoad(ofd2.FileName,RenderContext));
            }
        }
        #endregion

        //③スプライト描画を更新する処理をオーバーライドする。
        //BeginとEndの間で呼ばれるので、スプライトの更新は必ずこのメソッド内で行わなければならない。
        protected override void RenderSprite()
        {
            //③-A 文字を表示してみる
            SpriteBatch.DWRenderTarget.DrawText(string.Format("FPS:{0}",FpsCounter.FPS),textFormat,SpriteBatch.FillRectangle,colorBrush);
            /*説明:
             * MMFにおいて2D表示をする際は、SpriteBatch.DWRenderTarget内のメソッドを利用する。
             * DrawTextはその名の通り文字を出力するメソッドである。
             * 
             *∋- 第一引数：string 表示させたい文字
             * 上記の例では、string.Format("FPS:{0}",FpsCounter.FPS)に当たる
             * つまり、FPS:{0}の{0}の部分が、FpsCounter.FPSの値に置き換えられたものになるということである。
             * FpsCounterはRenderFormにおいて宣言されており、RenderFormお継承するクラスであれば利用可能である。
             * FpsCounter.FPSは最新10フレームの平均的な1秒あたりのフレームの描画回数を示すものである。
             * 
             *∋- 第二引数:TextFormat 使用するテキストフォーマット
             * 利用するテキストフォーマット。フォント、サイズ、太字指定...etcいろいろなことが指定できる。
             * 
             *∋- 第三引数:Rectangle 描画する領域
             * 任意の領域に対して、テキストを描画できる。与えられた範囲で、さらに揃えを実行される。
             * つまり、右揃えのテキストフォーマットを利用して画面全体を描画対象にすれば右端にテキストは描画される。
             * なお、画面いっぱいの領域を指定するには、SpriteBatch.FillRectangleを指定すればよい。
             * 
             *∋- 第四引数：Brush 使用するブラシ
             * SolidColorBrushをはじめとする様々なブラシを利用可能。
             * (MMFはまだすべてのブラシのリサイズなどを自動化などしていないので、作成されていないクラスについてはDirectWriteのクラスを直接扱うことになります。
             * Disposeの問題などに注意してください。アプリケーション終了時にフリーズやAccessViolationExceptionが発生する場合があります、)
             */

            //③-B 何か図形を描く
            SpriteBatch.DWRenderTarget.DrawRoundedRectangle(colorBrush,new RoundedRectangle(){Bottom = 60,Left = 20,RadiusX = 5,RadiusY = 10,Right = 90,Top = 10});
            //DrawRoundedRectangleは中身を塗りつぶさない、FillRoundedRectangleは中身を塗りつぶす。このように、Draw系は輪郭、Fill系は中身も塗りつぶす図形描画である。

            //③-C 画像を描画してみる
            SpriteBatch.DWRenderTarget.DrawBitmap(bitmap,Rectangle.FromLTRB(100,100,200,200));
        }


    }
}
