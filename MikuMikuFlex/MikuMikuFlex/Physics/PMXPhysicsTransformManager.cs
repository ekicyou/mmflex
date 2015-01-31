using System.Collections.Generic;
using System.Diagnostics;
using BulletSharp;
using MMDFileParser.PMXModelParser;
using MMDFileParser.PMXModelParser.JointParam;
using MMF.Bone;
using MMF.Model;
using MMF.Morph;
using SlimDX;
using IDisposable = System.IDisposable;

namespace MMF.Physics {

	/// <summary>
	/// 物理演算ボーンのマネージャ
	/// </summary>
	public class PMXPhysicsTransformManager:ITransformUpdater,IDisposable
    {

        /// <summary>
        /// 最初に計算しておいてあとで繰り返し使う剛体データ
        /// </summary>
		class TempRigidBodyData 
        {
            /// <summary>
            /// 初期位置
            /// </summary>
			public readonly Vector3 position;

            /// <summary>
            /// 初期姿勢行列
            /// </summary>
			public readonly Matrix init_matrix;

            /// <summary>
            /// オフセット行列
            /// </summary>
			public readonly Matrix offset_matrix;

            /// <summary>
            /// ボーンインデックス
            /// </summary>
			public readonly int boneIndex;
            
            /// <summary>
            /// 物理計算タイプ
            /// </summary>
			public readonly PhysicsCalcType physicsCalcType;
            
            /// <summary>
            /// 剛体形状
            /// </summary>
			public readonly RigidBodyShape shape;

            /// <summary>
            /// 剛体データのテンポラリ
            /// </summary>
            /// <param name="rigidBodyData">剛体データ</param>
			public TempRigidBodyData(RigidBodyData rigidBodyData)
            {
				position = rigidBodyData.Position;
				var r = rigidBodyData.Rotation;
				init_matrix = Matrix.RotationYawPitchRoll(r.Y, r.X, r.Z) * Matrix.Translation(position);
				offset_matrix = Matrix.Invert(init_matrix);
				boneIndex = rigidBodyData.BoneIndex;
				physicsCalcType = rigidBodyData.PhysicsCalcType;
				shape = rigidBodyData.Shape;
			}
		}

        /// <summary>
        /// ボーン
        /// </summary>
		private PMXBone[] bones;

        /// <summary>
        /// 剛体データのテンポラリリスト
        /// </summary>
        private List<TempRigidBodyData> tempRigidBodyData_s = new List<TempRigidBodyData>();
		
        /// <summary>
		/// Bullet物理演算
		/// </summary>
        private BulletManager bulletManager;
        
        /// <summary>
        /// Bulletの剛体リスト
        /// </summary>
        private List<BulletSharp.RigidBody> rigidBodies = new List<BulletSharp.RigidBody>();

	    private static bool physicsAsserted;

	    /// <summary>
        /// 剛体を生成する
        /// </summary>
        /// <param name="rigidBodyData_s">剛体データ</param>
        private void CreateRigid(List<RigidBodyData> rigidBodyData_s)
        {
            foreach (var r in rigidBodyData_s)
            {
                var tempRigidBodyData = new TempRigidBodyData(r);
                var init_matrix = tempRigidBodyData.init_matrix;
                tempRigidBodyData_s.Add(tempRigidBodyData);
                BulletSharp.RigidBody rigidBody = null;
                CollisionShape collisionShape;
                switch (r.Shape)
                {
                    case RigidBodyShape.Sphere: // 球体
                        collisionShape = new SphereShape(r.Size.X);
                        break;
                    case RigidBodyShape.Box:        // ボックス
                        collisionShape = new BoxShape(r.Size.X, r.Size.Y, r.Size.Z);
                        break;
                    case RigidBodyShape.Capsule:    // カプセル
                        collisionShape = new CapsuleShape(r.Size.X, r.Size.Y);
                        break;
                    default:    //  例外処理
                        throw new System.Exception("Invalid rigid body data");
                }
                var rigidProperty = new RigidProperty(r.Mass, r.Repulsion, r.Friction, r.MoveAttenuation, r.RotationAttenuation);
                var superProperty = new SuperProperty(r.PhysicsCalcType == PhysicsCalcType.Static,  (CollisionFilterGroups)(1 << r.RigidBodyGroup), (CollisionFilterGroups)r.UnCollisionGroupFlag);
                rigidBody = bulletManager.CreateRigidBody(collisionShape, init_matrix, rigidProperty, superProperty);
                rigidBodies.Add(rigidBody);
            }
        }

        /// <summary>
        /// ジョイント生成
        /// </summary>
        /// <param name="jointData_s">ジョイントデータリスト</param>
        private void CreateJoint(List<JointData> jointData_s) 
        {
			foreach (var jointData in jointData_s)
            {
				var jointParam = (Spring6DofJointParam)jointData.JointParam;
                var connectedBodyPair = CreateConnectedBodyPair(jointParam);
                var restriction = CreateRestriction(jointParam);
                var stiffness = CreateStiffness(jointParam);
                bulletManager.Add6DofSpringConstraint(connectedBodyPair, restriction, stiffness);
			}
		}

        /// <summary>
        /// 6軸ジョイントに繋がる剛体のペアを作る
        /// </summary>
        /// <param name="jointParam">6軸ジョイントパラメータ</param>
        /// <returns>6軸ジョイントに繋がる剛体のペア</returns>
        private Joint6ConnectedBodyPair CreateConnectedBodyPair(Spring6DofJointParam jointParam)
        {
            var bodyA = rigidBodies[jointParam.RigidBodyAIndex];
            var bodyAworld_inv = Matrix.Invert(bulletManager.GetWorld(bodyA));
            var bodyB = rigidBodies[jointParam.RigidBodyBIndex];
            var bodyBworld_inv = Matrix.Invert(bulletManager.GetWorld(bodyB));
            var jointRotation = jointParam.Rotation;
            var jointPosition = jointParam.Position;
            var jointWorld = Matrix.RotationYawPitchRoll(jointRotation.Y, jointRotation.X, jointRotation.Z) * Matrix.Translation(jointPosition.X, jointPosition.Y, jointPosition.Z);
            var connectedBodyA = new Joint6ConnectedBody(bodyA, jointWorld * bodyAworld_inv);
            var connectedBodyB = new Joint6ConnectedBody(bodyB, jointWorld * bodyBworld_inv);
            return new Joint6ConnectedBodyPair(connectedBodyA, connectedBodyB);
        }

        /// <summary>
        /// 6軸可動制限を作る
        /// </summary>
        /// <param name="jointParam">6軸ジョイントパラメータ</param>
        /// <returns>6軸可動制限</returns>
        private Joint6Restriction CreateRestriction(Spring6DofJointParam jointParam)
        {
            var movementRestriction = new Joint6MovementRestriction(jointParam.MoveLimitationMin, jointParam.MoveLimitationMax);
            var rotationRestriction = new Joint6RotationRestriction(jointParam.RotationLimitationMin, jointParam.RotationLimitationMax);
            return new Joint6Restriction(movementRestriction, rotationRestriction);
        }

        private Joint6Stiffness CreateStiffness(Spring6DofJointParam jointParam)
        {
            return new Joint6Stiffness(jointParam.SpringMoveCoefficient, jointParam.SpringRotationCoefficient);
        }

		/// <summary>
		/// コンストラクタ
		/// </summary>
		/// <param name="bones">ボーンリスト</param>
		/// <param name="rigidBodies">剛体リスト</param>
		public PMXPhysicsTransformManager(PMXBone[] bones, List<RigidBodyData> rigidBodyData_s, 
            List<JointData> jointData_s)
        {
			this.bones = bones;
			var gravity = new Vector3(0, -9.8f*2.5f, 0);
			bulletManager = new BulletManager(gravity);
			CreateRigid(rigidBodyData_s);
			CreateJoint(jointData_s);
		}

		/// <summary>
		/// リソースを開放する
		/// </summary>
		public void Dispose()
        {
            this.tempRigidBodyData_s.Clear();
            this.bulletManager.Dispose();
            this.rigidBodies.Clear();
		}

	    /// <summary>
	    /// ITransformUpdaterのメンバーの実装
	    /// </summary>
	    public bool UpdateTransform()
        {
            // 下記の処理では剛体タイプがボーン追従であるもの全てをボーン行列に設定し
            // 物理演算を行った後に剛体タイプが物理演算のものを設定、その後に剛体タイプが
            // 物理＋ボーン位置あわせの物を設定する。設定および計算を行う順番に注意すること。

            // ボーン追従タイプの剛体にボーン行列を設定
			for (int i = 0; i < rigidBodies.Count; ++i) 
            { 
				var t = tempRigidBodyData_s[i];
                // 関連ボーン有りで剛体タイプがボーン追従の場合
				if (t.boneIndex != -1 && t.physicsCalcType == PhysicsCalcType.Static)
                {
                    // ボーン行列を設定処理
                    var bone = bones[t.boneIndex];
					var rigidMat = t.init_matrix*bone.GlobalPose;
					bulletManager.MoveRigidBody(rigidBodies[i], rigidMat);
				}
			}
            // 物理演算シミュレーション
			bulletManager.StepSimulation();

            // 物理演算の結果に合わせてボーンを設定
			for (int i = 0; i <  rigidBodies.Count; ++i)
            {
				var t = tempRigidBodyData_s[i];
                // 関連ボーンなし時は処理しない
				if (t.boneIndex != -1)
                {
                    var bone = bones[t.boneIndex];
                    var globalPose = t.offset_matrix * bulletManager.GetWorld(rigidBodies[i]);
                    if (float.IsNaN(globalPose.M11))
                    {
                        if (!physicsAsserted)
                            Debug.WriteLine("物理演算の結果が不正な結果を出力しました。\nPMXの設定を見直してください。うまくモーション動作しない可能性があります。");
                        physicsAsserted = true;
                        continue;
                    }
                    var localPose = globalPose * Matrix.Invert(bone.Parent.GlobalPose);
                    var mat = Matrix.Translation(bone.Position) * localPose * Matrix.Translation(-bone.Position);
                    bone.Translation = new Vector3(mat.M41, mat.M42, mat.M43);
                    bone.Rotation = Quaternion.RotationMatrix(mat);
                    bone.UpdateGrobalPose();
				}
			}

            // ボーン位置あわせタイプの剛体の位置移動量にボーンの位置移動量を設定
			for (int i = 0; i < rigidBodies.Count; ++i)
            { 
				var t = tempRigidBodyData_s[i];
                // 関連ボーン有りで剛体タイプが物理＋ボーン位置あわせの場合ボーンの位置移動量を設定
				if (t.boneIndex != -1 && t.physicsCalcType == PhysicsCalcType.BoneAlignment)
                {
					var bone = bones[t.boneIndex];
					var v = new Vector3(bone.GlobalPose.M41, bone.GlobalPose.M42, bone.GlobalPose.M43);	// ボーンの移動量
					var p = new Vector3(t.init_matrix.M41, t.init_matrix.M42, t.init_matrix.M43) + v;
					var m = bulletManager.GetWorld(rigidBodies[i]);
					m.M41 = p.X; m.M42 = p.Y; m.M43 = p.Z;
					bulletManager.MoveRigidBody(rigidBodies[i], m);
				}
			}
            return false;
	    }


	}
}
