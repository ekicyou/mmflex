using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using BulletSharp;

namespace MMF.Physics
{
    /// <summary>
    /// Bulletの世界を作るクラス
    /// </summary>
    internal class DynamicsWorldFactory : System.IDisposable
    {
        /// <summary>
        /// 初期化用メンバー変数
        /// </summary>
        private readonly DefaultCollisionConfiguration collisionConfiguration;
        private readonly CollisionDispatcher dispatcher;
        private readonly BroadphaseInterface overlappingPairCache;
        private readonly SequentialImpulseConstraintSolver solver;

        /// <summary>
        /// コンストラクタ
        /// </summary>
        public DynamicsWorldFactory()
        {
            collisionConfiguration = new DefaultCollisionConfiguration();
            dispatcher = new CollisionDispatcher(collisionConfiguration);
            overlappingPairCache = new DbvtBroadphase();
            solver = new SequentialImpulseConstraintSolver();
        }

        /// Bulletの世界を作る
        /// </summary>
        /// <param name="gravity">重力</param>
        /// <returns>Bulletの世界</returns>
        public DiscreteDynamicsWorld CreateDynamicsWorld(SlimDX.Vector3 gravity)
        {
            var dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
            dynamicsWorld.Gravity = gravity;
            return dynamicsWorld;
        }

        /// <summary>
        /// リソースを開放する
        /// </summary>
        public void Dispose()
        {
            solver.Dispose();
            overlappingPairCache.Dispose();
            dispatcher.Dispose();
            collisionConfiguration.Dispose();
        }

    }
}
