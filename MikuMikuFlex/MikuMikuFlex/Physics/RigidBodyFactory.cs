using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using BulletSharp;
using SlimDX;

namespace MMF.Physics
{
    /// <summary>
    /// 剛体を作るクラス
    /// </summary>
    internal class RigidBodyFactory : System.IDisposable
    {
        /// <summary>
        ///リソース開放のため、作った 剛体を管理する配列
        /// </summary>
        private AlignedCollisionShapeArray collisionShapes = new AlignedCollisionShapeArray();

        /// <summary>
        /// 物理演算の世界
        /// </summary>
        private DiscreteDynamicsWorld dynamicsWorld;

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="dynamicsWorld">物理演算の世界</param>
        public RigidBodyFactory(DiscreteDynamicsWorld dynamicsWorld)
        {
            this.dynamicsWorld = dynamicsWorld;
        }

        /// <summary>
        /// 剛体を作る
        /// </summary>
        /// <param name="collisionShape">剛体の形</param>
        /// <param name="world">剛体のワールド変換行列</param>
        /// <param name="rigidProperty">剛体の物性</param>
        /// <param name="superProperty">物理演算を超越した特性</param>
        /// <returns></returns>
        public RigidBody CreateRigidBody(CollisionShape collisionShape, Matrix world, RigidProperty rigidProperty, SuperProperty superProperty)
        {
            var mass = superProperty.kinematic ? 0 : rigidProperty.mass;
            collisionShapes.Add(collisionShape);
            Vector3 localInertia = new Vector3(0, 0, 0);
            if (mass != 0) collisionShape.CalculateLocalInertia(mass, out localInertia);
            DefaultMotionState motionState = new DefaultMotionState(world);
            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, motionState, collisionShape, localInertia);
            RigidBody body = new RigidBody(rbInfo);
            body.Restitution = rigidProperty.restitution;
            body.Friction = rigidProperty.friction;
            body.SetDamping(rigidProperty.linear_damp, rigidProperty.angular_damp);
            float linearDamp = body.LinearDamping;
            float angularDamp = body.AngularDamping;
            if (superProperty.kinematic) body.CollisionFlags = body.CollisionFlags | CollisionFlags.KinematicObject;
            body.ActivationState = ActivationState.DisableDeactivation;
            dynamicsWorld.AddRigidBody(body, superProperty.group, superProperty.mask);
            return body;
        }

        /// <summary>
        /// リソースを開放する
        /// </summary>
        public void Dispose()
        {
            for (int i = dynamicsWorld.NumCollisionObjects - 1; i >= 0; --i)
            {
                CollisionObject obj = dynamicsWorld.CollisionObjectArray[i];
                RigidBody body = RigidBody.Upcast(obj);
                if (body != null && body.MotionState != null) body.MotionState.Dispose();
                dynamicsWorld.RemoveCollisionObject(obj);
                obj.Dispose();
            }
            for (int i = 0; i < collisionShapes.Count; ++i)
            {
                CollisionShape collisionShape = collisionShapes[i];
                collisionShapes[i] = null;
                collisionShape.Dispose();
            }
            collisionShapes.Dispose();
        }

    }
}
