using System;
/*
 * *****************************************************************************************************************************************************************
 * MMF Tutorials 01「Render 3DCG world to your form」
 * 
 * ◎What is purpose of this section?
 * 1,To learn how to use RenderForm of MMF.
 * 2,To learn how to loop paint method of MMF
 * 
 * ◎Required time
 * 5m
 * 
 * ◎Difficulty
 * Very easy
 * 
 * ◎Preparations
 * ・Add reference of MMF to your project file.
 * ・Add reference of SlimDX to your project file.(you should use x86 and .Net4.0 SlimDX assembly)
 * 
 * ◎Step of this tutorial
 * ①～②
 * ・Program.cs
 * ・Form1.cs
 * only two files above.
 * 
 * ◎Required Runtime
 * DirectX end user runtime
 * SlimDX end user runtime for .Net4.0 (x86)
 * .Net Framework 4.5
 * 
 * ◎Goal
 * To display the form painted light blue solid color.
 * 
 ********************************************************************************************************************************************************************/
namespace _01_Initialize3DCGToYourForm
{
    internal static class Program
    {
        /// <summary>
        /// Entry point of application. 
        ///  </summary>
        [STAThread]
        private static void Main()
        {
            //①Change message loop way of form

            //When you create "Windows form application" in C#,Visual studio may generate the code below.
            //Application.EnableVisualStyles();
            //Application.SetCompatibleTextRenderingDefault(false);
            //Application.Run(new Form1());

            //you should change this code to the code below.
            MMF.MessagePump.Run(new Form1());

            /*
             * ◎Description of the code above
             * 
             * Form1 extends RenderForm. RenderForm is the base class defined in MMF to render 3DCG world. 
             * RenderForm will render when you call Render method. Therefore,you should call Render method frequently.
             * void MMF.MessagePump.Run(RenderForm form);
             * the method above is use for calling frequently Render method of first argument.
             *
             * In default,you can toggle screen mode with Alt+Enter.
             */
        }
    }
}