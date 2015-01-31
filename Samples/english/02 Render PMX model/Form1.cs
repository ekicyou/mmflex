using System;
using System.Windows.Forms;
using MMF;
using MMF.CG.Model.MMD;

namespace _02_SimpleRenderPMX
{
    public partial class Form1 : RenderForm
    {
        public Form1()
        {
            InitializeComponent();
        }

        //①Overrides Form.OnLoad method
        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e); //You should call base.Load(e); at first line of this method when you override OnLoad method.

            //Use OpenFileDialog
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Filter = "pmx model file(*.pmx)|*.pmx";
            if (ofd.ShowDialog() == DialogResult.OK)
            {
            //Load model when ShowDialog result is DialogResult.OK


                //②Load Model
                MMDModel model = MMDModelWithPhysics.OpenLoad(ofd.FileName, RenderContext);//MMDModel MMDModelWithPhysics.OpenLoad(string fileName,RenderContext);
                //When you want to disable physics calclation,you can do that by using the static method below.
                //MMDModel model=MMDModel.OpenLoad(ofd.FileName, RenderContext);
                //RenderContext contains device data,direct2D device data..and so on using for rendering 3DCG world. If the class  extends RenderForm,you can use this field as RenderContext.
                //The method or classes rendering something sometimes require this value.

                //③Add model to world.
                WorldSpace.AddResource(model);
                //WorldSpace manages the models added to world. If you add a drawable obeject to WorldSpace,the object will be rendered. 
            }
        }
    }
}