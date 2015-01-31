using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MMF;
using MMF.CG;
using MMF.CG.Camera.CameraMotion;
using MMF.CG.Model.Grid;

namespace PhysicsTest3 {
	public partial class Form1 : RenderForm {

		Billiard billiard;
		int frame = 0;

		public Form1() {
			InitializeComponent();
		}

	    protected override void OnLoad(EventArgs e)
	    {
	        base.OnLoad(e);
            ScreenContext.CameraMotionProvider = new BasicCameraControllerMotionProvider(this, this);
            Dock = DockStyle.Fill;
            var grid = new BasicGrid();
            grid.Visibility = true;
            grid.Load(RenderContext);
            WorldSpace.AddResource(grid);
            billiard = new Billiard(RenderContext);
            var models = billiard.GetModels();
            foreach (var model in models) WorldSpace.AddResource(model);
	    }

	    private void Form1_Load(object sender, EventArgs e) {

		}

		private void Form1_FormClosing(object sender, FormClosingEventArgs e) {
			billiard.Dispose();	// bulletのコンポーネントを先に廃棄しないとエラーになる
		}

		private void Form1_Paint(object sender, PaintEventArgs e) {
		}

		private void timer1_Tick(object sender, EventArgs e) {
			if (frame == 150) billiard.Shot();
			billiard.Run();
			++frame;
		}
	}
}
