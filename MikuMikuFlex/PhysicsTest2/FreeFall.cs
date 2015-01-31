using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMDD.CG.Physics;
using MMDD.CG.Model.MMD;
using BulletSharp;
using SlimDX;

namespace PhysicsTest2 {
	class FreeFall {

		BulletPhysics bulletPhysics;
		List<RigidBody> bodies = new List<RigidBody>();
		List<MMDModel> models = new List<MMDModel>();
		RigidBody ground;
		Matrix model_world_from_rigid;	// 剛体からみたモデルのワールド変換行列

		// 剛体からみたモデルのワールド変換行列を取得
		Matrix GetModelWorldFromRigid() {
			var mmdRigidBody = models[0].Model.rigidBodyList.RigidBodies[0];
			var pos = mmdRigidBody.Position;	// モデルからみた剛体の位置
			var rot = mmdRigidBody.Rotation;	// モデルからみた剛体の回転
			var rigid_world_from_model = Matrix.RotationYawPitchRoll(rot.Y, rot.X, rot.Z)*Matrix.Translation(pos);	// モデルからみた剛体のワールド変換行列
			return Matrix.Invert(rigid_world_from_model);
		}

		// 玉を作る
		void CreateSphereRigids() {
			var mmdRigidBody = models[0].Model.rigidBodyList.RigidBodies[0];
			var radius = mmdRigidBody.Size.X;
			var mass = mmdRigidBody.Mass;
			var restitution = mmdRigidBody.Repulsion;
			var friction = mmdRigidBody.Friction;
			var linear_damp = mmdRigidBody.MoveAttenuation;
			var angular_damp = mmdRigidBody.RotationAttenuation;
			for (int i = 0; i < models.Count; ++i) {
				var world = Matrix.Translation(i*0.003f, 1.0f*i + 1.0f, i*0.002f);	// 剛体のワールド変換行列
				bodies.Add(bulletPhysics.CreateSphere(radius, world, mass, restitution, friction, linear_damp, angular_damp));
			}
		}

		// 土台を作る
		void CreateBoxRigid() {
			const float width = 150, height = 0.5f, depth = 150;
			const float mass = 0;
			const float restitution = 1.0f;
			var world = Matrix.Translation(0, -height, 0);	// 上面がy = 0になるように置く
			ground = bulletPhysics.CreateBox(width, height, depth, world, mass, restitution);
		}

		// コンストラクタ
		public FreeFall(List<MMDModel> models) {
			this.models = models;
			var gravity = new Vector3(0, -9.8f*2.5f, 0);
			bulletPhysics = new BulletPhysics(gravity);
			model_world_from_rigid = GetModelWorldFromRigid();
			CreateSphereRigids();
			CreateBoxRigid();
		}

		// 物理演算でモデルの配置を更新
		public void Run() { 
			bulletPhysics.StepSimulation();
			for (int i = 0; i < models.Count; ++i) {
				var rigid_world = bulletPhysics.GetWorld(bodies[i]);	// 剛体のワールド変換行列
				var model_world = model_world_from_rigid*rigid_world;	// モデルのワールド変換行列
				var v = model_world.get_Rows(3);
				models[i].Transformer.Position = new Vector3(v.X, v.Y, v.Z);
				models[i].Transformer.Rotation = Quaternion.RotationMatrix(model_world);
			}
		}
	}
}
