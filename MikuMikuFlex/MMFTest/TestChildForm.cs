using MMF;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MMF.Controls.Forms;
using MMF.Sprite.D2D;
using SlimDX.DirectWrite;

namespace CGTest
{
    public partial class TestChildForm :D2DSupportedRenderForm
    {
        private D2DSpriteSolidColorBrush brush;
        private D2DSpriteTextformat format;

        public TestChildForm(RenderContext context):base(context)
        {
            InitializeComponent();

        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            //format = SpriteBatch.CreateTextformat("Meiriyo", 30, FontWeight.ExtraBold);
            //brush = SpriteBatch.CreateSolidColorBrush(Color.Aquamarine);
        }

        protected override void RenderSprite()
        {
          // SpriteBatch.DWRenderTarget.DrawText("Test", format.Format, SpriteBatch.FillRectangle, brush);
        }
    }
}
