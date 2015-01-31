using System;
using MMF;
/*
 * *****************************************************************************************************************************************************************
 * MMF Tutorial 03「Apply motion to PMX model」
 * 
 * ◎What is the purpose of this section?
 * 1,To learn how to apply motion to PMX by using MMF.
 * 
 * ◎Rquired time
 * 5m
 * 
 * ◎Difficulty
 * Very easy
 * 
 * ◎Preperation
 ・You should create the project implemented as 01 sample code.
 * ★★★Important★★★→You should copy x86,x64,Toon and Shader directories included Sample directory into bin\Debug directory.  
 * 
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
 * PMX model moved by vmd motion data is rendered.
 * 
 ********************************************************************************************************************************************************************/
using _03_ApplyMotionToPMX;

namespace _03_ApplyMotionToPMX
{
    internal static class Program
    {
        /// <summary>
        /// Entry point of application
        /// </summary>
        [STAThread]
        private static void Main()
        {
            MessagePump.Run(new Form1());
        }
    }
}