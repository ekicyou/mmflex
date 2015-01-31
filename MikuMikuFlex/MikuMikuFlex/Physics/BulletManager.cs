using System.Collections.Generic;
using BulletSharp;
using SlimDX;

namespace MMF.Physics
{
    /// <summary>
    /// 物理演算のラッパークラス
    /// </summary>
    internal class BulletManager : System.IDisposable
    {
        /// <summary>
        /// Bulletの世界
        /// </summary>
        private DiscreteDynamicsWorld dynamicsWorld;

        /// <summary>
        /// Bulletの世界を作るクラス
        /// </summary>
        private DynamicsWorldFactory dynamicsWorldFactory = new DynamicsWorldFactory();

        /// <summary>
        /// 剛体を作るクラス
        /// </summary>
        private RigidBodyFactory rigidBodyFactory;

        /// <summary>
        /// 拘束を作るクラス
        /// </summary>
        private ConstraintFactory constraintFactory;

        /// <summary>
        /// 経過時間を計るクラス
        /// </summary>
        private BulletTimer bulletTimer = new BulletTimer();

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="gravity">重力</param>
        public BulletManager(Vector3 gravity)
        {
            dynamicsWorld = dynamicsWorldFactory.CreateDynamicsWorld(gravity);
            rigidBodyFactory = new RigidBodyFactory(dynamicsWorld);
            constraintFactory = new ConstraintFactory(dynamicsWorld);
        }

        /// <summary>
        /// リソースを開放したか否か
        /// </summary>
        private bool isDisposed = false;

        /// <summary>
        /// リソースを開放する
        /// </summary>
        public void Dispose()
        {
            if (isDisposed) return;
            constraintFactory.Dispose();
            rigidBodyFactory.Dispose();
            dynamicsWorld.Dispose();
            dynamicsWorldFactory.Dispose();
            isDisposed = true;
        }

        /// <summary>
        /// 剛体を作る
        /// </summary>
        /// <param name="collisionShape">剛体の形</param>
        /// <param name="world">剛体のワールド変換行列</param>
        /// <param name="rigidProperty">剛体の物性</param>
        /// <param name="superProperty">物理演算を超越した特性</param>
        /// <returns>剛体</returns>
        public RigidBody CreateRigidBody(CollisionShape collisionShape, Matrix world, RigidProperty rigidProperty, SuperProperty superProperty)
        {
            return rigidBodyFactory.CreateRigidBody(collisionShape, world, rigidProperty, superProperty);
        }

        /// <summary>
        /// 剛体に空間上の点への拘束を追加
        /// </summary>
        /// <param name="body">剛体</param>
        /// <param name="pivot">拘束する点</param>
        public void AddPointToPointConstraint(RigidBody body, ref Vector3 pivot)
        {
            constraintFactory.AddPointToPointConstraint(body, ref pivot);
        }

        /// <summary>
        /// 剛体と剛体の間に点拘束を追加
        /// </summary>
        /// <param name="bodyA">剛体A</param>
        /// <param name="bodyB">剛体B</param>
        /// <param name="pivotInA">剛体Aから見た拘束点の位置</param>
        /// <param name="pivotInB">剛体Bから見た拘束点の位置</param>
        public void AddPointToPointConstraint(RigidBody bodyA, RigidBody bodyB, ref Vector3 pivotInA, ref Vector3 pivotInB)
        {
            constraintFactory.AddPointToPointConstraint(bodyA, bodyB, ref pivotInA, ref pivotInB);
        }

        /// <summary>
        /// 剛体と剛体の間に6軸バネ拘束を追加
        /// </summary>
        /// <param name="connectedBodyPair">繋ぐ剛体のペア</param>
        /// <param name="restriction">6軸可動制限</param>
        /// <param name="stiffness">6軸バネ</param>
        public void Add6DofSpringConstraint(Joint6ConnectedBodyPair connectedBodyPair, Joint6Restriction restriction, Joint6Stiffness stiffness)
        {
            constraintFactory.Add6DofSpringConstraint(connectedBodyPair, restriction, stiffness);
        }

        /// <summary>
        /// キネマティック剛体を移動する
        /// </summary>
        /// <param name="body">剛体</param>
        /// <param name="world">ワールド変換行列</param>
        public void MoveRigidBody(RigidBody body, Matrix world)
        {
            body.MotionState.WorldTransform = world;
        }

        /// <summary>
        /// 物理演算の世界の時間を進める 
        /// </summary>
        public void StepSimulation()
        {
            var elapsedTime = bulletTimer.GetElapsedTime(); //[ms]
            dynamicsWorld.StepSimulation(elapsedTime / 1000f, 10);
        }

        /// <summary>
        /// 指定した剛体について、物理演算結果のワールド行列を取得 する
        /// </summary>
        /// <param name="body">剛体</param>
        /// <returns>ワールド変換行列</returns>
        public Matrix GetWorld(RigidBody body)
        {
            return body.MotionState.WorldTransform;
        }

    }
}