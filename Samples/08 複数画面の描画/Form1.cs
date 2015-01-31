using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MMF;
using MMF.Controls.Forms;

namespace _08_MultiScreenRendering
{
    public partial class Form1 : D2DSupportedRenderForm
    {
        private ChildForm childForm;

        public Form1()
        {
            InitializeComponent();
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            
            //①-A MMFを利用する別のフォームをのインスタンスを作成する。
            childForm = new ChildForm(RenderContext);
            childForm.Show();
            //①-B ワールド空間を更新対象としてRenderContextに追加する。このとき、WorldSpaceはOnLoadでインスタンスが作成されているので、Show以降に追加する必要がある。
            
            if (MessageBox.Show("共通のワールド空間を利用しますか?", "", MessageBoxButtons.YesNo)==DialogResult.Yes)
            {
                childForm.ScreenContext.WorldSpace = ScreenContext.WorldSpace;//childSpaceのワールド空間をこのForm1のワールド空間と同一のものにする。
                //この場合はRenderContextを指定しないで初期化されたフォームであるこのForm1のWorldSpaceは標準で更新対象になるため
                //UpdateReqireWorldsに追加する必要はない。
                //こうした場合、二つのスクリーンは同じワールド空間を表示する。
            }
            else
            {
                RenderContext.UpdateRequireWorlds.Add(childForm.WorldSpace);//標準のワールド空間以外は更新するよう指定する必要がある。
            }
            ControllerForm controller=new ControllerForm(this
                ,childForm);
            controller.Show();
        }

        protected override void RenderSprite()
        {
            
        }

        protected override void OnPresented()
        {
            base.OnPresented();
            //①-C 描画の瞬間はここでは、Form1の描画直後にしておく。
            //ただし、同じRenderContextで行われる描画に対して同時にRenderは呼び出すことができない。
            childForm.Render();//childFormのRenderを呼び出すために、描画後呼び出されるOnPresentedメソッドをオーバーライドする。
        }
    }
}
