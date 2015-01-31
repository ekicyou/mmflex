using System.Drawing;
using System.Windows.Forms;
using MMF.Matricies.Projection;
using SlimDX;

namespace MMF.Matricies.Camera.CameraMotion
{
    public class BasicCameraControllerMotionProvider:ICameraMotionProvider
    {
        private Vector3 xAxis=new Vector3(1,0,0);
        private Vector3 yAxis=new Vector3(0,1,0);

        public BasicCameraControllerMotionProvider(Control control,Control wheelRevieveControl,float initialDistance=45f)
        {
            distance = initialDistance;
            cameraPositionRotation = Quaternion.Identity;
            control.MouseDown += panel_MouseDown;
            control.MouseMove += panel_MouseMove;
            control.MouseUp += panel_MouseUp;
            wheelRevieveControl.MouseWheel += wheelRevieveControl_MouseWheel;
            MouseWheelSensibility = 2.0f;
            RightButtonRotationSensibility = 0.005f;
            MiddleButtonTranslationSensibility = 0.01f;
        }

        void wheelRevieveControl_MouseWheel(object sender, MouseEventArgs e)
        {
            if (e.Delta > 0)
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
        private Quaternion cameraPositionRotation { get; set; }

        private Vector2 cameraLookatTranslation { get; set; }

        private float distance { get; set; }

        public float MouseWheelSensibility { get; set; }

        public float RightButtonRotationSensibility { get; set; }

        public float MiddleButtonTranslationSensibility { get; set; }

        private void panel_MouseMove(object sender, MouseEventArgs e)
        {
            int x = e.Location.X - LastMousePosition.X;
            int y = e.Location.Y - LastMousePosition.Y;
            if (isRightMousePushed)
            {
//回転量については累積で記録
                cameraPositionRotation *= Quaternion.RotationAxis(yAxis, RightButtonRotationSensibility*x)*
                                          Quaternion.RotationAxis(xAxis, RightButtonRotationSensibility*(-y));
                cameraPositionRotation.Normalize();
            }
            if (isMiddleMousePushed)
            {//変化量を記録しておく
                cameraLookatTranslation+=new Vector2(x,y)*MiddleButtonTranslationSensibility;
            }
            LastMousePosition = e.Location;
        }


        private void panel_MouseUp(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Right) isRightMousePushed = false;
            if (e.Button == MouseButtons.Middle) isMiddleMousePushed = false;
        }

        private void panel_MouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Right) isRightMousePushed = true;
            if (e.Button == MouseButtons.Middle) isMiddleMousePushed = true;
        }


        void ICameraMotionProvider.UpdateCamera(CameraProvider cp1, IProjectionMatrixProvider proj)
        {
            Vector3 cp2la = Vector3.TransformCoordinate(new Vector3(0, 0, 1),
                Matrix.RotationQuaternion(cameraPositionRotation));
            xAxis = Vector3.Cross(cp2la, cp1.CameraUpVec);
            xAxis.Normalize();
            yAxis = Vector3.Cross(xAxis, cp2la);
            yAxis.Normalize();
            cp1.CameraLookAt += xAxis*cameraLookatTranslation.X + yAxis*cameraLookatTranslation.Y;
            cp1.CameraPosition = cp1.CameraLookAt + distance*(-cp2la);
            cameraLookatTranslation = Vector2.Zero;
        }
    }
}
