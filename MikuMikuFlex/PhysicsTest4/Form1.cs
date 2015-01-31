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
using MMF.CG.Model.MMD;
using MMF.CG.Motion;
using MMF.CG.Skinning;
using MMF.CG.Physics;

namespace PhysicsTest4 {
	public partial class Form1 : Form {
		D2DSupportedRenderControl panel= new D2DSupportedRenderControl();

		public Form1() {
			InitializeComponent();
		}

		private void Form1_Load(object sender, EventArgs e) {
			panel.Initialize();
			panel.RenderContext.CurrentTargetContext.CameraMotionProvider=new BasicCameraControllerMotionProvider(panel, this);
			panel.Dock = DockStyle.Fill;
			Controls.Add(panel);
	
			var grid = new BasicGrid();
			grid.Visibility = true;
			grid.Load(panel.RenderContext);
            panel.RenderContext.CurrentTargetContext.WorldSpace.AddResource(grid);

			var model = MMDModelWithPhysics.OpenLoad("../../res/reimu.pmx", panel.RenderContext);
            panel.RenderContext.CurrentTargetContext.WorldSpace.AddResource(model);

            var currentMotion = model.MotionManager.AddMotionFromFile("../../res/love&joy.vmd", false);
			model.MotionManager.ApplyMotion(currentMotion, 0, ActionAfterMotion.Replay);
		}

		private void Form1_FormClosing(object sender, FormClosingEventArgs e) {
			panel.Dispose();
		}

		private void Form1_Paint(object sender, PaintEventArgs e) {
			if (panel.IsInitialized) panel.Render();
		}

		private void timer1_Tick(object sender, EventArgs e) {
			Invalidate();
		}
	}
}
