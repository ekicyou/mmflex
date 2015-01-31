using System;
using System.Windows.Forms;
using MMF;

/*
 * *****************************************************************************************************************************************************************
 * MMF Tutorial 02「Render PMX model」
 * 
 * ◎What the purpose of this section?
 * 1,To learn how to render PMX model by using MMF.
 * 
 * ◎Required time
 * 5m
 * 
 * ◎Difficulty
 * Very easy
 * 
 * ◎Preparation
 * ・You should create the project implemented as 01 sample code.
 * ★★★Important★★★→You should copy x86,x64,Toon and Shader directories included Sample directory into bin\Debug directory. * 
 * ◎Steps of this tutorial
 * ①～③
 * ・Form1.cs
 * only the code above.
 * 
 * ◎Reqired runtime
 * DirectX enduser runtime
 * SlimDX end user runtime .Netframework 4.5
 * 
 * ◎Goal
 * When you can render pmx model in your form.
 * 
 ********************************************************************************************************************************************************************/

namespace _02_SimpleRenderPMX
{
    internal static class Program
    {
        /// <summary>
        ///     Entry point of application.
        /// </summary>
        [STAThread]
        private static void Main()
        {
            MessagePump.Run(new Form1());
        }
    }
}