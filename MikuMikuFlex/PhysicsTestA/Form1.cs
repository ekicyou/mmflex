#region

using System;
using System.Diagnostics;
using System.Drawing;
using BulletSharp;
using MMDFileParser.PMXModelParser;
using MMF;
using MMF.CG.Camera.CameraMotion;
using MMF.CG.Model.Grid;
using MMF.CG.Model.MMD;
using MMF.CG.Model.Sprite.D2D;
using MMF.CG.Physics;
using SlimDX;
using SlimDX.DirectWrite;

#endregion

namespace PhysicsTestA
{
    public partial class Form1 : D2DSupportedRenderForm
    {
        public RigidBody ball;
        public RigidBody ball2;
        public MMDModel ball_Model;
        public MMDModel ball_Model2;
        public BulletPhysics bulletPhysics;
        private D2DSpriteSolidColorBrush brush;
        private D2DSpriteTextformat format;

        public RigidBody ground;
        private Matrix rigid2model;
        private Matrix rigid22model;
        public RigidBody[] walls = new RigidBody[5];
        private float distance;
        ContactCallBack callBack=new ContactCallBack();

        public Form1()
        {
            InitializeComponent();
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            ScreenContext.CameraMotionProvider = new BasicCameraControllerMotionProvider(this, this);
            BasicGrid grid = new BasicGrid();
            grid.Load(RenderContext);
            WorldSpace.AddResource(grid);
            bulletPhysics = new BulletPhysics(new Vector3(0, -9.8f, 0f));
            ground = bulletPhysics.CreatePlane(0, Matrix.Identity);
            ball_Model = MMDModel.OpenLoad("1.pmx", RenderContext);
            ball_Model2 = MMDModel.OpenLoad("1.pmx", RenderContext);
            RigidBodyData data = ball_Model.Model.RigidBodyList.RigidBodies[0];
            ball = bulletPhysics.CreateSphere(data.Size.X, Matrix.Translation(-15, 12f, 0), 1f, 1f, 0f);
            ball2 = bulletPhysics.CreateSphere(data.Size.X, Matrix.Translation(15, 12f, 0), 1f, 1f, 0f);
            
            rigid2model = GetModelWorldFromRigid();

            WorldSpace.AddResource(ball_Model);
            WorldSpace.AddResource(ball_Model2);
            CreateWalls();
            ball.ApplyCentralImpulse(new Vector3(2, 0.01f, 0f));
            ball2.ApplyCentralImpulse(new Vector3(-2,0,0));
            brush = SpriteBatch.CreateSolidColorBrush(Color.Brown);
            format = SpriteBatch.CreateTextformat("Meiriyo", 30);
            format.Format.ParagraphAlignment=ParagraphAlignment.Center;
            format.Format.TextAlignment=TextAlignment.Center;
            timer1.Start();
        }

        private void CreateWalls()
        {
            walls[0] = bulletPhysics.CreatePlane(0,
                Matrix.RotationAxis(new Vector3(1, 0, 0), (float) Math.PI)*Matrix.Translation(0, 20, 0));
            walls[1] = bulletPhysics.CreatePlane(0,
                Matrix.RotationAxis(new Vector3(1, 0, 0), (float) - (Math.PI/2))*Matrix.Translation(0, 0, 20));
            walls[2] = bulletPhysics.CreatePlane(0,
                Matrix.RotationAxis(new Vector3(1, 0, 0), (float) (Math.PI/2))*Matrix.Translation(0, 0, -20));
            walls[3] = bulletPhysics.CreatePlane(0,
                Matrix.RotationAxis(new Vector3(0, 0, 1), (float) (Math.PI/2))*Matrix.Translation(20, 0, 0));
            walls[4] = bulletPhysics.CreatePlane(0,
                Matrix.RotationAxis(new Vector3(0, 0, 1), (float) -(Math.PI/2))*Matrix.Translation(-20, 0, 0));
        }

        private Matrix GetModelWorldFromRigid()
        {
            RigidBodyData mmdRigidBody = ball_Model.Model.RigidBodyList.RigidBodies[0];
            Vector3 pos = mmdRigidBody.Position; // モデルからみた剛体の位置
            Vector3 rot = mmdRigidBody.Rotation; // モデルからみた剛体の回転
            Matrix rigid_world_from_model = Matrix.RotationYawPitchRoll(rot.Y, rot.X, rot.Z)*Matrix.Translation(pos);
                // モデルからみた剛体のワールド変換行列
            return Matrix.Invert(rigid_world_from_model);
        }

        public override void OnUpdated()
        {
            base.OnUpdated();
        }

        protected override void OnHandleDestroyed(EventArgs e)
        {
            base.OnHandleDestroyed(e);
            bulletPhysics.Dispose();
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            bulletPhysics.StepSimulation();
            bulletPhysics.dynamicsWorld.ContactPairTest(ball, ball2, callBack);
            Matrix rigid_world = bulletPhysics.GetWorld(ball);
            Matrix model_world = rigid2model*rigid_world;
            Vector4 transLation = model_world.get_Rows(3);
            ball_Model.Transformer.Position = new Vector3(transLation.X, transLation.Y, transLation.Z);
            ball_Model.Transformer.Rotation = Quaternion.RotationMatrix(model_world);
             rigid_world = bulletPhysics.GetWorld(ball2);
             model_world = rigid2model * rigid_world;
             transLation = model_world.get_Rows(3);
            ball_Model2.Transformer.Position = new Vector3(transLation.X, transLation.Y, transLation.Z);
            ball_Model2.Transformer.Rotation = Quaternion.RotationMatrix(model_world);
            
        }

        protected override void RenderSprite()
        {
            
            SpriteBatch.DWRenderTarget.DrawText(string.Format("{0}",callBack.Hitcount),format,SpriteBatch.FillRectangle,brush);
        }
    }

    class ContactCallBack:CollisionWorld.ContactResultCallback
    {
        public int Hitcount = 0;

        public override float AddSingleResult(ManifoldPoint cp, CollisionObjectWrapper colObj0Wrap, int partId0, int index0,
            CollisionObjectWrapper colObj1Wrap, int partId1, int index1)
        {
            Hitcount++;
            return 0;
        }
    }
}