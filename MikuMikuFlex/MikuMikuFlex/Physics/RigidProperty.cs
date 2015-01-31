using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMF.Physics
{
    /// <summary>
    /// 剛体の物性
    /// </summary>
    internal class RigidProperty
    {
        /// <summary>
        /// 質量
        /// </summary>
        public float mass { private set; get; }

        /// <summary>
        /// 反発係数
        /// </summary>
        public float restitution { private set; get; }

        /// <summary>
        /// 摩擦係数
        /// </summary>
        public float friction { private set; get; }

        /// <summary>
        /// 移動減衰係数
        /// </summary>
        public float linear_damp { private set; get; }

        /// <summary>
        /// 回転減衰係数
        /// </summary>
        public float angular_damp { private set; get; }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="mass">質量。0にすると動かないstatic剛体になる。</param>
        /// <param name="restitution">反発係数</param>
        /// <param name="friction">摩擦係数</param>
        /// <param name="linear_damp">移動減衰係数</param>
        /// <param name="angular_damp">回転減衰係数</param>
        public RigidProperty(float mass = 0, float restitution = 0, float friction = 0.5f, float linear_damp = 0, float angular_damp = 0)
        {
            this.mass = mass;
            this.restitution = restitution;
            this.friction = friction;
            this.linear_damp = linear_damp;
            this.angular_damp = angular_damp;
        }
    }

}
