using SlimDX;

namespace MMF.Model
{
    public class BasicTransformer : ITransformer
    {

        private Vector3 foward;
        private Vector3 initialFoward;
        private Vector3 initialTop;
        private Quaternion rotation;
        private Vector3 top;

        private Matrix localTransform;
        private Vector3 position;
        private Vector3 scale;

        public BasicTransformer(Vector3 top, Vector3 forward)
        {
            InitialTop = top;
            InitialFoward = forward;
            Reset();
        }

        public BasicTransformer() : this(new Vector3(0, 1, 0), new Vector3(0, 0, -1))
        {
        }

        public Vector3 Position

        {
            get { return position; }
            set
            {
                position = value; 
                CalcLocalTransform();
            }
        }

        public Quaternion Rotation
        {
            get { return rotation; }
            set
            {
                rotation = value;
                rotation.Normalize();
                Foward = Vector3.TransformCoordinate(InitialFoward, Matrix.RotationQuaternion(Rotation));
                Top = Vector3.TransformCoordinate(InitialTop, Matrix.RotationQuaternion(Rotation));
                Top.Normalize();
                Foward.Normalize();
                CalcLocalTransform();
            }
        }

        public Vector3 Scale
        {
            get { return scale; }
            set
            {
                scale = value; 
                CalcLocalTransform();
            }
        }

        public Vector3 Foward
        {
            get { return foward; }
            set
            {
                foward = value;
                foward.Normalize();
            }
        }

        public Vector3 Top
        {
            get { return top; }
            set
            {
                top = value;
                top.Normalize();
            }
        }


        public Vector3 InitialTop
        {
            get { return initialTop; }
            private set
            {
                initialTop = value;
                initialTop.Normalize();
            }
        }

        public Vector3 InitialFoward
        {
            get { return initialFoward; }
            private set
            {
                initialFoward = value;
                InitialFoward.Normalize();
                
            }
        }

        public void Reset()
        {
            Top = initialTop;
            Foward = initialFoward;
            Rotation = Quaternion.Identity;
            Position = Vector3.Zero;
            Scale = new Vector3(1f);
            CalcLocalTransform();
        }

        private void CalcLocalTransform()
        {
            localTransform = Matrix.Scaling(Scale)*Matrix.RotationQuaternion(Rotation)*Matrix.Translation(Position);
        }

        public Matrix LocalTransform
        {
            get { return localTransform; }
            private set { localTransform = value; }
        }
    }
}