using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MMDD.CG;
using MMDD.CG.Camera.CameraMotion;
using MMDD.CG.Grid;
using MMDD.CG.Model.MMD;
using MMDD.CG.Motion;
using SlimDX;

namespace PhysicsTest2 {
	public partial class Form1 : Form {
		
		D2DSupportedRenderPanel panel= new D2DSupportedRenderPanel();
		FreeFall freeFall;

		public Form1() {
			InitializeComponent();
		}

		private void Form1_Load(object sender, EventArgs e) {
			panel.Initialize();
			panel.RenderContext.CameraMotionProvider=new BasicCameraControllerMotionProvider(panel, this);
			panel.Dock = DockStyle.Fill;
			Controls.Add(panel);
			var grid = new NormalGrid();
			grid.Visiblity = true;
			grid.Load(panel.RenderContext);
			panel.RenderContext.AddResource(grid);
			const int Nball = 20;	// 球の数
			var models = new List<MMDModel>();
			for (int i = 0; i < Nball; ++i) {
				models.Add(MMDModel.OpenLoad("../../res/1.pmx", panel.RenderContext));
				panel.RenderContext.AddResource(models[i]);
			}
			freeFall = new FreeFall(models);
		}

		private void Form1_FormClosing(object sender, FormClosingEventArgs e) {
			panel.Dispose();
		}

		private void Form1_Paint(object sender, PaintEventArgs e) {
			if (panel.IsInitialized) panel.Render();
		}

		private void timer1_Tick(object sender, EventArgs e) {
			freeFall.Run();
			Invalidate();
		}
	}
}
