using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SlimDX;
using BulletSharp;

namespace MMF.Physics
{
    /// <summary>
    /// 6軸ジョイントにつながる剛体
    /// </summary>
    internal class Joint6ConnectedBody
    {
        /// <summary>
        /// 剛体
        /// </summary>
        public RigidBody rigidBody { private set; get; }

        /// <summary>
        /// ワールド変換行列
        /// </summary>
        public Matrix world { private set; get; }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="rigidBody">剛体</param>
        /// <param name="world">ジョイントのワールド変換行列(剛体のローカル座標系)</param>
        public Joint6ConnectedBody(RigidBody rigidBody, Matrix world)
        {
            this.rigidBody = rigidBody;
            this.world = world;
        }
    }

    /// <summary>
    /// 6軸ジョイントの移動制限
    /// </summary>
    internal class Joint6MovementRestriction
    {
        /// <summary>
        /// 移動制限1
        /// </summary>
        public Vector3 c_p1 { private set; get; }

        /// <summary>
        /// 移動制限2
        /// </summary>
        public Vector3 c_p2 { private set; get; }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="c_p1">移動制限1</param>
        /// <param name="c_p2">移動制限2</param>
        public Joint6MovementRestriction(Vector3 c_p1, Vector3 c_p2)
        {
            this.c_p1 = c_p1;
            this.c_p2 = c_p2;
        }
    }

    /// <summary>
    /// 6軸ジョイントの回転制限
    /// </summary>
    internal class Joint6RotationRestriction
    {
        /// <summary>
        /// 回転制限1
        /// </summary>
        public Vector3 c_r1 { private set; get; }

        /// <summary>
        /// 回転制限2
        /// </summary>
        public Vector3 c_r2 { private set; get; }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="c_r1">回転制限1</param>
        /// <param name="c_r2">回転制限2</param>
        public Joint6RotationRestriction(Vector3 c_r1, Vector3 c_r2)
        {
            this.c_r1 = c_r1;
            this.c_r2 = c_r2;
        }
    }

    /// <summary>
    /// バネ剛性
    /// </summary>
    internal class Joint6Stiffness
    {
        /// <summary>
        /// 平行移動成分
        /// </summary>
        public Vector3 translation { private set; get; }

        /// <summary>
        /// 回転移動成分
        /// </summary>
        public Vector3 rotation { private set; get; }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="translation">平行移動成分</param>
        /// <param name="rotation">回転移動成分</param>
        public Joint6Stiffness(Vector3 translation, Vector3 rotation)
        {
            this.translation = translation;
            this.rotation = rotation;
        }
    }

    /// <summary>
    /// 6軸ジョイントにつながる剛体のペア
    /// </summary>
    internal class Joint6ConnectedBodyPair
    {
        /// <summary>
        /// 6軸ジョイントにつながる剛体A
        /// </summary>
        public Joint6ConnectedBody connectedBodyA { private set; get; }

        /// <summary>
        /// 6軸ジョイントにつながる剛体B
        /// </summary>
        public Joint6ConnectedBody connectedBodyB { private set; get; }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="connectedBodyA">6軸ジョイントにつながる剛体A</param>
        /// <param name="connectedBodyB">6軸ジョイントにつながる剛体B</param>
        public Joint6ConnectedBodyPair(Joint6ConnectedBody connectedBodyA, Joint6ConnectedBody connectedBodyB)
        {
            this.connectedBodyA = connectedBodyA;
            this.connectedBodyB = connectedBodyB;
        }
    }

    /// <summary>
    /// 6軸可動制限
    /// </summary>
    internal class Joint6Restriction
    {
        /// <summary>
        /// 移動制限
        /// </summary>
        public Joint6MovementRestriction movementRestriction { private set; get; }

        /// <summary>
        /// 回転制限
        /// </summary>
        public Joint6RotationRestriction rotationRestriction { private set; get; }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="movementRestriction">移動制限</param>
        /// <param name="rotationRestriction">回転制限</param>
        public Joint6Restriction(Joint6MovementRestriction movementRestriction, Joint6RotationRestriction rotationRestriction)
        {
            this.movementRestriction = movementRestriction;
            this.rotationRestriction = rotationRestriction;
        }
    }

}
