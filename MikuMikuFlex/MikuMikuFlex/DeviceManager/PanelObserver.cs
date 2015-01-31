using System.Windows.Forms;
using MMF.Motion;
using SlimDX;

namespace MMF.DeviceManager
{
    /// <summary>
    /// 主にマウスなどのパネル上の動きを監視するクラス
    /// MMEエフェクトのため
    /// </summary>
    public class PanelObserver
    {
        public Vector2 MousePosition { get; private set; }

        public Vector4 LeftMouseDown { get; private set; }

        public Vector4 MiddleMouseDown { get; private set; }

        public Vector4 RightMouseDown { get; private set; }

        /// <summary>
        /// 監視対象のコントロール
        /// </summary>
        private Control RelatedControl { get; set; }

        /// <summary>
        /// MMEのマウス機能が有効かどうか
        /// </summary>
        public bool IsMMEMouseEnable { get; set; }

        public PanelObserver(Control control)
        {
            RelatedControl = control;
            IsMMEMouseEnable = false;
            control.MouseMove += MouseHandler;
            control.MouseDown += MouseHandler;
        }

        void MouseHandler(object sender, System.Windows.Forms.MouseEventArgs e)
        {
            if (IsMMEMouseEnable)
            {
                float x = 0, y = 0;
                float leftT=LeftMouseDown.W, middleT=MiddleMouseDown.W, rightT=RightMouseDown.W;
                float leftP = 0f, middleP = 0f, rightP = 0f;
                if (e.X!=0)
                {
                    x = (float)e.X*2f/(float)RelatedControl.Width - 1f;
                }
                if (e.Y != 0)
                {
                    y = (float)e.Y * 2f / (float)RelatedControl.Height - 1f;
                }
                MousePosition = new Vector2(x, y);
                if (e.Button.HasFlag(MouseButtons.Left))
                {
                    leftT = MotionTimer.stopWatch.ElapsedMilliseconds/1000f;
                    leftP = 1;
                }
                if (e.Button.HasFlag(MouseButtons.Middle))
                {
                    middleT = MotionTimer.stopWatch.ElapsedMilliseconds/1000f;
                    middleP = 1;
                }
                if (e.Button.HasFlag(MouseButtons.Right))
                {
                    rightT = MotionTimer.stopWatch.ElapsedMilliseconds/1000f;
                    rightP = 1;
                }
                LeftMouseDown=new Vector4(x,y,leftP,leftT);
                MiddleMouseDown = new Vector4(x, y, middleP, middleT);
                RightMouseDown = new Vector4(x, y, rightP, rightT);
            }
        }
    }
}
