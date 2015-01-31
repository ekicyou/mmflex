using System.Drawing;
using System.Windows.Controls;
using System.Windows.Input;
using MMF.Matricies.Projection;
using SlimDX;

namespace MMF.Matricies.Camera.CameraMotion
{
    public class WPFBasicCameraControllerMotionProvider:ICameraMotionProvider
    {
        public bool isLocked;

        private Vector3 xAxis=new Vector3(1,0,0);
        private Vector3 yAxis=new Vector3(0,1,0);
        private Control control;

        public WPFBasicCameraControllerMotionProvider(Control control,float initialDistance=45f)
        {
            distance = initialDistance;
            cameraPositionRotation = Quaternion.Identity;
            this.control = control;
            control.MouseDown += panel_MouseDown;
            control.MouseMove += panel_MouseMove;
            control.MouseUp += panel_MouseUp;
            control.MouseWheel += wheelRevieveControl_MouseWheel;
            MouseWheelSensibility = 2.0f;
            RightButtonRotationSensibility = 5f;
            MiddleButtonTranslationSensibility = 1f;
        }

        void wheelRevieveControl_MouseWheel(object sender, MouseWheelEventArgs mouseWheelEventArgs)
        {
            if(isLocked)return;
            if (mouseWheelEventArgs.Delta > 0)
            {
                distance -= MouseWheelSensibility;
                if (distance <= 0)
                    distance = 0.0001f;
            }
            else
            {
                distance += MouseWheelSensibility;
            }
        }

        /// <summary>
        /// マウスの前回の位置
        /// </summary>
        private Point LastMousePosition { get; set; }

        /// <summary>
        /// マウスの右ボタンが押されているか
        /// </summary>
        private bool isRightMousePushed { get; set; }

        /// <summary>
        /// マウスの中ボタンが押されているか
        /// </summary>
        private bool isMiddleMousePushed { get; set; }

        /// <summary>
        /// カメラの注視点を中心とする回転量
        /// </summary>
        protected Quaternion cameraPositionRotation { get; set; }

        protected Vector2 cameraLookatTranslation { get; set; }

        protected Vector3 cameraLookatTranslationOfWorld { get; set; }

        protected float distance { get; set; }

        public float MouseWheelSensibility { get; set; }

        public float RightButtonRotationSensibility { get; set; }

        public float MiddleButtonTranslationSensibility { get; set; }

        private void panel_MouseMove(object sender, System.Windows.Input.MouseEventArgs mouseEventArgs)
        {
            if (isLocked) return;
            int x = (int) (mouseEventArgs.GetPosition(control).X - LastMousePosition.X);
            int y = (int) (mouseEventArgs.GetPosition(control).Y - LastMousePosition.Y);
            if (isRightMousePushed)
            {
//回転量については累積で記録
                cameraPositionRotation *= Quaternion.RotationAxis(yAxis, RightButtonRotationSensibility/1000f*x)*
                                          Quaternion.RotationAxis(xAxis, RightButtonRotationSensibility/1000f*(-y));
                cameraPositionRotation.Normalize();
            }
            if (isMiddleMousePushed)
            {//変化量を記録しておく
                cameraLookatTranslation+=new Vector2(x,y)*MiddleButtonTranslationSensibility/100f;
            }
            LastMousePosition = new Point((int) mouseEventArgs.GetPosition(control).X, (int) mouseEventArgs.GetPosition(control).Y);
        }


        private void panel_MouseUp(object sender, MouseButtonEventArgs mouseButtonEventArgs)
        {
            if (mouseButtonEventArgs.RightButton==MouseButtonState.Released) isRightMousePushed = false;
            if (mouseButtonEventArgs.MiddleButton==MouseButtonState.Released) isMiddleMousePushed = false;
        }

        private void panel_MouseDown(object sender, MouseButtonEventArgs mouseButtonEventArgs)
        {
            if (mouseButtonEventArgs.RightButton == MouseButtonState.Pressed) isRightMousePushed = true;
            if (mouseButtonEventArgs.MiddleButton == MouseButtonState.Pressed) isMiddleMousePushed = true;
        }


        public virtual void UpdateCamera(CameraProvider cp1, IProjectionMatrixProvider proj)
        {
            Vector3 cp2la = Vector3.TransformCoordinate(new Vector3(0, 0, 1),
                Matrix.RotationQuaternion(cameraPositionRotation));
            xAxis = Vector3.Cross(cp2la, cp1.CameraUpVec);
            xAxis.Normalize();
            yAxis = Vector3.Cross(xAxis, cp2la);
            yAxis.Normalize();
            cp1.CameraLookAt += xAxis*cameraLookatTranslation.X + yAxis*cameraLookatTranslation.Y;
            cp1.CameraLookAt += cameraLookatTranslationOfWorld;
            cp1.CameraPosition = cp1.CameraLookAt + distance*(-cp2la);
            cameraLookatTranslation = Vector2.Zero;
            cameraLookatTranslationOfWorld = Vector3.Zero;
        }
    }
}
