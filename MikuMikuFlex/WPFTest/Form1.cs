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
using MMF.CG.DeviceManager;
using MMF.CG.MMEEffect.VariableSubscriber.MaterialSubscriber;
using MMF.CG.Model.Grid;
using MMF.CG.Model.Shape;
using SlimDX.Direct3D11;
using SlimDX.DXGI;

namespace WPFTest
{
    public partial class Form1 : RenderForm
    {
        public TextureTargetContext textureTargetContext;

        private PlaneBoard board;
        private MainWindow mainWindow;

        public Form1()
        {
            InitializeComponent();
            
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            textureTargetContext=new WPFTargetTextureContext(RenderContext,new Size(800,800),new SampleDescription(1,0));
            RenderContext.UpdateRequireWorlds.Add(textureTargetContext.WorldSpace);
            
             var resourceView = new ShaderResourceView(RenderContext.DeviceManager.Device, textureTargetContext.RenderTarget);
            board=new PlaneBoard(RenderContext,resourceView);
            WorldSpace.AddResource(board);

            BasicGrid grid=new BasicGrid();
            grid.Load(RenderContext);
            textureTargetContext.WorldSpace.AddResource(grid);

            mainWindow = new MainWindow();
            mainWindow.Show();
        }

        public override void Render()
        {
            base.Render();
        }
    }
}
