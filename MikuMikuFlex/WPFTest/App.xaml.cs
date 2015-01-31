using System;
using System.Collections.Generic;
using System.Configuration;
using System.Data;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using MMF;
using MessagePump=SlimDX.Windows.MessagePump;
namespace WPFTest
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {
        private void Application_Startup(object sender, StartupEventArgs e)
        {
            //Form1 form1 = new Form1();
            //form1.Show();
            //MessagePump.Run(form1, form1.Render);
            MainWindow mainWindow = new MainWindow();
            mainWindow.Show();
            //MessagePump.Run(mainWindow.Render);
        }
    }
}
