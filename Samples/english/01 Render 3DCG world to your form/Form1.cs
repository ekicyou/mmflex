namespace _01_Initialize3DCGToYourForm
{

    //②Change base class of your form.
    //When you create "Windows form aplication",Visual Studio may generate Form1 class definition as below.
    //public partial class Form1:Form
    //You should change this coe as below.
    public partial class Form1 : MMF.RenderForm
    {
        /*
         * ◎Description
         * When you want to draw 3DCG world,it is the fastest way to extends MMF.RenderForm.
         * MMF.RenderForm extends Form class,overides some method appropriately.
         * (Notice)When you override OnPaint method,you should call base.OnPaint(e); at first line of OnPaint method.
         */
        public Form1()
        {
            InitializeComponent();
            /*
             * Extra
             * How to chage background color?
             * When you change background color of your form,you should change BackgroundColor property as the code below.
             * You should pass the value as new Vector3(Red,Green,Blue);
             */
            //Example
            //BackgroundColor = new SlimDX.Vector3(1f, 0f, 0f);
        }

    }
}
