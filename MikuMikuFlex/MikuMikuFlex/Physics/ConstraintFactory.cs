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
    /// 拘束を作るクラス
    /// </summary>
    internal class ConstraintFactory : System.IDisposable
    {
        /// <summary>
        /// 物理演算の世界
        /// </summary>
        private DiscreteDynamicsWorld dynamicsWorld;

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="dynamicsWorld">物理演算の世界</param>
        public ConstraintFactory(DiscreteDynamicsWorld dynamicsWorld)
        {
            this.dynamicsWorld = dynamicsWorld;
        }

        /// <summary>
        /// 剛体に空間上の点への拘束を追加
        /// </summary>
        /// <param name="body">剛体</param>
        /// <param name="pivot">拘束する点</param>
        public void AddPointToPointConstraint(RigidBody body, ref Vector3 pivot)
        {
            dynamicsWorld.AddConstraint(new Point2PointConstraint(body, pivot));
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
            dynamicsWorld.AddConstraint(new Point2PointConstraint(bodyA, bodyB, pivotInA, pivotInB));
        }

        /// <summary>
        /// 剛体と剛体の間に6軸バネ拘束を追加
        /// </summary>
        /// <param name="connectedBodyPair">繋ぐ剛体のペア</param>
        /// <param name="restriction">6軸可動制限</param>
        /// <param name="stiffness">6軸バネ</param>
        public void Add6DofSpringConstraint(Joint6ConnectedBodyPair connectedBodyPair, Joint6Restriction restriction, Joint6Stiffness stiffness)
        {
            var bodyA = connectedBodyPair.connectedBodyA.rigidBody;
            var bodyB = connectedBodyPair.connectedBodyB.rigidBody;
            var frameInA = connectedBodyPair.connectedBodyA.world;
            var frameInB = connectedBodyPair.connectedBodyB.world;
            var constraint = new Generic6DofSpringConstraint(bodyA, bodyB, frameInA, frameInB, true); // 第五引数の効果は謎。どちらでも同じ様に見える……。

            var c_p1 = restriction.movementRestriction.c_p1;
            var c_p2 = restriction.movementRestriction.c_p2;
            var c_r1 = restriction.rotationRestriction.c_r1;
            var c_r2 = restriction.rotationRestriction.c_r2;
            constraint.LinearLowerLimit = new Vector3(c_p1.X, c_p1.Y, c_p1.Z); // 型はベクトルだがベクトル量ではないのでZは反転しない。
            constraint.LinearUpperLimit = new Vector3(c_p2.X, c_p2.Y, c_p2.Z);
            constraint.AngularLowerLimit = new Vector3(c_r1.X, c_r1.Y, c_r1.Z);
            constraint.AngularUpperLimit = new Vector3(c_r2.X, c_r2.Y, c_r2.Z);

            SetStiffness(stiffness.translation.X, 0, constraint);
            SetStiffness(stiffness.translation.Y, 1, constraint);
            SetStiffness(stiffness.translation.Z, 2, constraint);
            SetStiffness(stiffness.rotation.X, 3, constraint);
            SetStiffness(stiffness.rotation.Y, 4, constraint);
            SetStiffness(stiffness.rotation.Z, 5, constraint);

            dynamicsWorld.AddConstraint(constraint);
        }

        /// <summary>
        /// 拘束にある一つの自由度へのバネをセットする
        /// </summary>
        /// <param name="stiffness">バネの値</param>
        /// <param name="index">自由度の種類を指定するインデックス(0~5。平行移動X, Y, Z, 回転移動X, Y, Zの順)</param>
        /// <param name="constraint">拘束</param>
        private void SetStiffness(float stiffness, int index, Generic6DofSpringConstraint constraint)
        {
            if (stiffness == 0.0f) return;
            constraint.EnableSpring(index, true);
            constraint.SetStiffness(index, stiffness);
        }

        /// <summary>
        /// リソースを開放する
        /// </summary>
        public void Dispose()
        {
            for (int i = dynamicsWorld.NumConstraints - 1; i >= 0; --i)
            {
                var constraint = dynamicsWorld.GetConstraint(i);
                dynamicsWorld.RemoveConstraint(constraint);
                constraint.Dispose();
            }
        }


    }
}
