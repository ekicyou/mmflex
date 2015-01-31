using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMDD.CG.Physics;
using BulletSharp;
using SlimDX;

namespace PhysicsTest {
	class FreeFall {
		BulletPhysics bulletPhysics;
		RigidBody sphere;
		RigidBody box;
		RigidBody bar;
		const float height = 5.0f;
		//const float radius = 0.4f;
		const float bar_width = 0.35f;
		const float bar_height = 0.35f;

		//// 玉を作る
		//void CreateSphereRigid() {
		//	const float mass = 1.0f;
		//	const float restitution = 0.8f;
		//	const float friction = 0.5f;
		//	const float linear_damp = 0.15f;
		//	const float angular_damp = 0.15f;
		//	var world = Matrix.Translation(0, height, 0);
		//	sphere = bulletPhysics.CreateSphere(radius, world, mass, restitution, friction, linear_damp, angular_damp);
		//}

		// 角棒を作る
		void CreateBarRigid() {
			const float depth = 5.0f;
			const float mass = 1.0f;
			const float restitution = 0.75f;
			const float friction = 0.5f;
			const float linear_damp = 0.30f;
			const float angular_damp = 0.30f;
			var trans = Matrix.Translation(0, height, 0);
			var rotation = Matrix.RotationZ(-(float)Math.PI/3);
			var world = rotation*trans;
			bar = bulletPhysics.CreateBox(bar_width, bar_height, depth, world, mass, restitution, friction, linear_damp, angular_damp);
		}

		// 土台を作る
		void CreateBoxRigid() {
			const float width = 5, height = 5, depth = 5;
			const float mass = 0;
			const float restitution = 1.0f;
			var world = Matrix.Translation(0, -height, 0);	// 上面がy = 0になるように置く
			box = bulletPhysics.CreateBox(width, height, depth, world, mass, restitution);
		}

		public FreeFall() {
			var gravity = new Vector3(0, -9.8f, 0);
			bulletPhysics = new BulletPhysics(gravity);
			CreateBarRigid();
			CreateBoxRigid();
		}

		public void Run() { bulletPhysics.StepSimulation();	}

		public Vector4 GetPosition() { return bulletPhysics.GetWorld(bar).get_Rows(3); }

		public float GetAngle() {
			Matrix world = bulletPhysics.GetWorld(bar);
			var v = new Vector3(1, 0, 0);
			var v1 = Vector3.TransformNormal(v, world);
			var v2 = Vector3.Cross(v, v1);
			var s  = 1;
			if (v2.Z < 0) s = -1;
			return s*(float)(Math.Acos((double)Vector3.Dot(v, v1)));
		}

//		public float GetRadius() { return radius; }

		public float GetBarWidth() { return bar_width*2; }

		public float GetBarHeight() { return bar_height*2; }
	}
}
