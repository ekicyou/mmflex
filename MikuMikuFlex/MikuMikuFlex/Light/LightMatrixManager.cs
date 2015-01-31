using MMF.Matricies;
using MMF.Matricies.Camera;
using MMF.Matricies.Projection;
using SlimDX;

namespace MMF.Light
{
    public class LightMatrixManager
    {
        public LightMatrixManager(MatrixManager manager)
        {
            this.manager = manager;
            Camera = new BasicCamera(new Vector3(0, 0, -20), new Vector3(0, 0, 0), new Vector3(0, 1, 0));
            Projection=new BasicProjectionMatrixProvider();
        }

        private MatrixManager manager;
        private Vector3 direction;

        public CameraProvider Camera { get; private set; }

        public BasicProjectionMatrixProvider Projection { get; private set; }

        public Vector3 Position
        {
            get
            {
                return Camera.CameraPosition;
            }
            set
            {
                Camera.CameraPosition = value;
                UpdateDirection();
            }
        }

        private void UpdateDirection()
        {
            direction = Vector3.Normalize(-Camera.CameraPosition);
        }

        public Vector3 Direction
        {
            get { return direction; }
            private set { direction = value; }
        }
    }
}
