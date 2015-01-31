#region

using System.Collections.Generic;
using BulletSharp;
using MMDFileParser.PMXModelParser;
using MMF.CG;
using MMF.CG.Model.MMD;
using MMF.CG.Physics;
using SlimDX;
using IDisposable = System.IDisposable;

#endregion

namespace PhysicsTest3
{
    internal class Billiard : IDisposable
    {
        private readonly List<RigidBody> balls = new List<RigidBody>();
        private readonly BulletPhysics bulletPhysics;
        private readonly List<MMDModel> models = new List<MMDModel>();
        private readonly Matrix transferMatrixFromRigidToModel; // ball剛体からみたballモデルのワールド変換行列(全ballで共通)。
        private List<Vector3> ballRayout;
        private int count;

        /// <summary>
        ///     コンストラクタ
        /// </summary>
        /// <param name="renderContext">レンダーコンテキスト</param>
        public Billiard(RenderContext renderContext)
        {
            CreateBallRayout();
            Vector3 gravity = new Vector3(0, -9.8f*2.5f, 0);
            bulletPhysics = new BulletPhysics(gravity);

            MMDModel model = MMDModel.OpenLoad("../../res/ビリヤード台.pmx", renderContext);
            models.Add(model);
            CreateBilliardRigids(model);

            for (int i = 0; i < ballRayout.Count; ++i)
            {
                model = MMDModel.OpenLoad("../../res/" + (i + 1) + ".pmx", renderContext);
                models.Add(model);
                CreateSphereRigid(model, ballRayout[i]);
            }
            transferMatrixFromRigidToModel = GetTransferMatrixFromRigidToModel();
        }

        /// <summary>
        ///     アンマネージなBulletを手動で捨てる
        /// </summary>
        public void Dispose()
        {
            bulletPhysics.Dispose();
        }

        // これをballの剛体用ワールド変換行列に左からかけることでモデル用ワールド変換行列に変換できる。

        // 球のレイアウト配列を作る
        private void CreateBallRayout()
        {
            const float height = 9.76f;
            const float x0 = 8.0f;
            const float root3 = 1.7320508f;
            const float r = 0.36f + 0.001f;
            ballRayout = new List<Vector3>
            {
                new Vector3(x0, height, 0),
                new Vector3(x0 + root3*r, height, r),
                new Vector3(x0 + root3*r, height, -r),
                new Vector3(x0 + 2*root3*r, height, 2*r),
                new Vector3(x0 + 2*root3*r, height, 0),
                new Vector3(x0 + 2*root3*r, height, -2*r),
                new Vector3(x0 + 3*root3*r, height, r),
                new Vector3(x0 + 3*root3*r, height, -r),
                new Vector3(x0 + 4*root3*r, height, 0),
                new Vector3(-1.5f*x0, height, 0)
            };
        }

        // transferMatrixFromRigidToModelを取得
        private Matrix GetTransferMatrixFromRigidToModel()
        {
            RigidBodyData mmdRigidBody = models[1].Model.RigidBodyList.RigidBodies[0];
            Vector3 pos = mmdRigidBody.Position; // モデルからみた剛体の位置
            Vector3 rot = mmdRigidBody.Rotation; // モデルからみた剛体の回転
            Matrix rigid_world_from_model = Matrix.RotationYawPitchRoll(rot.Y, rot.X, rot.Z)*Matrix.Translation(pos);
                // モデルからみた剛体のワールド変換行列
            return Matrix.Invert(rigid_world_from_model);
        }

        // 球の剛体を作る
        private void CreateSphereRigid(MMDModel model, Vector3 rigid_position)
        {
            RigidBodyData rigidBody = model.Model.RigidBodyList.RigidBodies[0];
            float radius = rigidBody.Size.X;
            float mass = rigidBody.Mass;
            float restitution = 1;//rigidBody.Repulsion;
            float friction = 0;//rigidBody.Friction;
            float linear_damp = 0;// rigidBody.MoveAttenuation;
            float angular_damp = rigidBody.RotationAttenuation;
            Matrix world = Matrix.Translation(rigid_position);
            ConvexShape shape;
            balls.Add(bulletPhysics.CreateSphere(radius, world, mass, restitution, friction, linear_damp, angular_damp));
            count++;
        }

        // ビリヤード台の剛体を作る
        private void CreateBilliardRigids(MMDModel model)
        {
            foreach (RigidBodyData rigidBody in model.Model.RigidBodyList.RigidBodies)
            {
                Matrix trans = Matrix.Translation(rigidBody.Position);
                Matrix rot = Matrix.RotationYawPitchRoll(rigidBody.Rotation[1], rigidBody.Rotation[0],
                    rigidBody.Rotation[2]);
                bulletPhysics.CreateBox(rigidBody.Size.X, rigidBody.Size.Y, rigidBody.Size.Z, rot*trans, 0,
                    1,0,0,0);
            }
            models[0].Transformer.Position = new Vector3(0, 0, 0);
            models[0].Transformer.Rotation = Quaternion.Identity;
        }

        /// <summary>
        ///     モデルを取得
        /// </summary>
        /// <returns>モデル</returns>
        public List<MMDModel> GetModels()
        {
            return models;
        }

        /// <summary>
        ///     物理演算でモデルの配置を更新
        /// </summary>
        public void Run()
        {
            bulletPhysics.StepSimulation();
            for (int i = 0; i < balls.Count; ++i)
            {
                Matrix rigid_world = bulletPhysics.GetWorld(balls[i]); // ball剛体のワールド変換行列
                Matrix model_world = transferMatrixFromRigidToModel*rigid_world; // ballモデルのワールド変換行列
                Vector4 v = model_world.get_Rows(3);
                models[1 + i].Transformer.Position = new Vector3(v.X, v.Y, v.Z);
                models[1 + i].Transformer.Rotation = Quaternion.RotationMatrix(model_world);
            }
        }

        /// <summary>
        ///     白玉をショット
        /// </summary>
        public void Shot()
        {
            balls[9].ActivationState = ActivationState.ActiveTag; // 剛体は動かないまま一定時間たつとスリープ状態になるので起こす必要がある
            balls[9].ApplyCentralImpulse(new Vector3(50.0f, -0f, 0f));
        }
    }
}