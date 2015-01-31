using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace PhysicsTest {
	public partial class Form1 : Form {

		FreeFall freeFall = new FreeFall();

		public Form1() {
			InitializeComponent();
		}

		private void timer1_Tick(object sender, EventArgs e) {
			freeFall.Run();
			Invalidate();
		}

		private void Form1_Paint(object sender, PaintEventArgs e) {
			const float scale = 50.0f;
			var origin = new Point(140, 260);
			var pos = freeFall.GetPosition();
			var bar_width = freeFall.GetBarWidth();
			var bar_height = freeFall.GetBarHeight();
			var angle = -180/(float)Math.PI*freeFall.GetAngle();
			//e.Graphics.FillEllipse(Brushes.Green, origin.X - radius*scale, origin.Y - radius*scale - pos.Y*scale, 2*radius*scale, 2*radius*scale);
			e.Graphics.TranslateTransform(origin.X + pos.X*scale, origin.Y - pos.Y*scale);
			e.Graphics.RotateTransform(angle);
			e.Graphics.FillRectangle(Brushes.Green, -bar_width*scale/2, -bar_height*scale/2, bar_width*scale, bar_height*scale);
		}
	}
}
