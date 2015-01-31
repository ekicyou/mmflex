using MMF.Matricies.Projection;
using SlimDX;

namespace MMF.Matricies.Camera.CameraMotion
{
    public class SideCameraMotionProvider : ICameraMotionProvider
    {
        private readonly ICameraMotionProvider _motionProvider;
        private readonly Quaternion _rotation;

        public SideCameraMotionProvider(ICameraMotionProvider motionProvider,Quaternion rotation)
        {
            _motionProvider = motionProvider;
            _rotation = rotation;
        }

        public void UpdateCamera(CameraProvider cp, IProjectionMatrixProvider proj)
        {
            _motionProvider.UpdateCamera(cp,proj);
            Vector3 la2cp = cp.CameraPosition - cp.CameraLookAt;
            Vector3 la2cp2 = Vector3.TransformNormal(la2cp, Matrix.RotationQuaternion(_rotation));
            cp.CameraPosition = cp.CameraLookAt + la2cp2;
        }
    }
}