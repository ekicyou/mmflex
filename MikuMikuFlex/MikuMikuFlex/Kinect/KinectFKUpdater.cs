using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows.Media.Media3D;
using MMF.Bone;
using MMF.Model;
using MMF.Morph;
using NiTEWrapper;
using SlimDX;
using Quaternion = SlimDX.Quaternion;
using Skeleton = NiTEWrapper.Skeleton;

namespace MMF.Kinect
{
    public class KinectFKUpdater : ITransformUpdater
    {
        private static Dictionary<SkeletonJoint.JointType,SkeletonJoint.JointType> ParentPairDictionary=new Dictionary<SkeletonJoint.JointType, SkeletonJoint.JointType>();

        /// <summary>
        /// ボーンの配列
        /// </summary>
        private PMXBone[] bones;

        static KinectFKUpdater()
        {
          ParentPairDictionary.Add(SkeletonJoint.JointType.HEAD,SkeletonJoint.JointType.NECK);
          ParentPairDictionary.Add(SkeletonJoint.JointType.LEFT_SHOULDER, SkeletonJoint.JointType.NECK);
          ParentPairDictionary.Add(SkeletonJoint.JointType.RIGHT_SHOULDER, SkeletonJoint.JointType.NECK);
          ParentPairDictionary.Add(SkeletonJoint.JointType.LEFT_ELBOW,SkeletonJoint.JointType.LEFT_SHOULDER);
          ParentPairDictionary.Add(SkeletonJoint.JointType.LEFT_HAND,SkeletonJoint.JointType.LEFT_ELBOW);
          ParentPairDictionary.Add(SkeletonJoint.JointType.RIGHT_ELBOW, SkeletonJoint.JointType.RIGHT_SHOULDER);
           ParentPairDictionary.Add(SkeletonJoint.JointType.RIGHT_HAND, SkeletonJoint.JointType.RIGHT_ELBOW);
          ParentPairDictionary.Add(SkeletonJoint.JointType.TORSO, SkeletonJoint.JointType.NECK);
            ParentPairDictionary.Add(SkeletonJoint.JointType.RIGHT_HIP, SkeletonJoint.JointType.TORSO);
            ParentPairDictionary.Add(SkeletonJoint.JointType.LEFT_HIP, SkeletonJoint.JointType.TORSO);
            ParentPairDictionary.Add(SkeletonJoint.JointType.LEFT_KNEE,SkeletonJoint.JointType.LEFT_HIP);
            ParentPairDictionary.Add(SkeletonJoint.JointType.RIGHT_KNEE, SkeletonJoint.JointType.RIGHT_HIP);
            ParentPairDictionary.Add(SkeletonJoint.JointType.RIGHT_FOOT, SkeletonJoint.JointType.RIGHT_KNEE);
            ParentPairDictionary.Add(SkeletonJoint.JointType.LEFT_FOOT, SkeletonJoint.JointType.LEFT_KNEE);
        }

        public Dictionary<SkeletonJoint.JointType,string> BindDictionary { get; set; }

        private KinectDeviceManager device;

        public Skeleton trackTarget;

        public short CurrentTrackUserId;


        public void StartTracking(short userId)
        {
            NiTE.Status status=device.NiteUserTracker.StartSkeletonTracking(userId);
            //device.TrackedUsers.Add(userId,device.CurrentUserTrackerFrameRef.getUserById(userId));
            CurrentTrackUserId = userId;
            Debug.WriteLine(status.ToString());
        }


        public event EventHandler<UserData> TrackingUser;

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="device">キネクトデバイスマネージャー</param>
        /// <param name="bones">ボーンの配列</param>
        public KinectFKUpdater(KinectDeviceManager device, PMXBone[] bones)
        {
            this.device = device;
            this.bones = bones;
            BindDictionary = new Dictionary<SkeletonJoint.JointType, string>();
            BindDictionary.Add(SkeletonJoint.JointType.RIGHT_SHOULDER,"右腕");
            BindDictionary.Add(SkeletonJoint.JointType.RIGHT_KNEE,"右ひじ");

        }

        /// <summary>
        /// ITransformUpdaterのメンバーの実装
        /// </summary>
        public bool UpdateTransform()
        {
            UserTrackerFrameRef usrFrameRef = device.CurrentUserTrackerFrameRef;
            if (CurrentTrackUserId >= 0)
            {
                UserData[] usrs =
                    (from sk in usrFrameRef.Users where sk.UserId == CurrentTrackUserId select sk).ToArray();
                if (usrs.Length != 1) return true;
                Skeleton skeleton = usrs[0].Skeleton;
                if (skeleton.State == Skeleton.SkeletonState.TRACKED)
                {
                    trackTarget = skeleton;
                    if (TrackingUser != null) TrackingUser(this, usrs[0]);
                    PMXBone head = getBone("頭");
                    PMXBone neck = getBone("首");
                    PMXBone torso = getBone("上半身");
                    PMXBone r_shoulder = getBone("右腕");
                    PMXBone r_elbow = getBone("右ひじ");
                    PMXBone r_hand = getBone("右手首");
                    PMXBone r_hip = getBone("右足");
                    PMXBone r_knee = getBone("右ひざ");
                    PMXBone r_foot = getBone("右足首");
                    PMXBone l_shoulder = getBone("左腕");
                    PMXBone l_elbow = getBone("左ひじ");
                    PMXBone l_hand = getBone("左手首");
                    PMXBone l_hip = getBone("左足");
                    PMXBone l_knee = getBone("左ひざ");
                    PMXBone l_foot = getBone("左足首");
                    SkeletonJoint sj_head = skeleton.getJoint(SkeletonJoint.JointType.HEAD);
                    SkeletonJoint sj_neck = skeleton.getJoint(SkeletonJoint.JointType.NECK);
                    SkeletonJoint sj_rShoulder = skeleton.getJoint(SkeletonJoint.JointType.RIGHT_SHOULDER);
                    SkeletonJoint sj_lShoulder = skeleton.getJoint(SkeletonJoint.JointType.LEFT_SHOULDER);
                    SkeletonJoint sj_rElbow = skeleton.getJoint(SkeletonJoint.JointType.RIGHT_ELBOW);
                    SkeletonJoint sj_lElbow = skeleton.getJoint(SkeletonJoint.JointType.LEFT_ELBOW);
                    SkeletonJoint sj_r_hand = skeleton.getJoint(SkeletonJoint.JointType.RIGHT_HAND);
                    SkeletonJoint sj_l_hand = skeleton.getJoint(SkeletonJoint.JointType.LEFT_HAND);
                    SkeletonJoint sj_torso = skeleton.getJoint(SkeletonJoint.JointType.TORSO);
                    SkeletonJoint sj_rHip = skeleton.getJoint(SkeletonJoint.JointType.RIGHT_HIP);
                    SkeletonJoint sj_lHip = skeleton.getJoint(SkeletonJoint.JointType.LEFT_HIP);
                    SkeletonJoint sj_rKnee = skeleton.getJoint(SkeletonJoint.JointType.RIGHT_KNEE);
                    SkeletonJoint sj_lKnee = skeleton.getJoint(SkeletonJoint.JointType.LEFT_KNEE);
                    SkeletonJoint sj_rFoot = skeleton.getJoint(SkeletonJoint.JointType.RIGHT_FOOT);
                    SkeletonJoint sj_lFoot = skeleton.getJoint(SkeletonJoint.JointType.LEFT_FOOT);
                    //腰のひねり
                    Vector3 shoulder_l2r =
                        Vector3.Normalize(
                            ToVector3(sj_rShoulder.Position) -
                            ToVector3(sj_lShoulder.Position));
                    Vector3 hip_l2r =
                        Vector3.Normalize(
                            ToVector3(sj_rHip.Position) -
                            ToVector3(sj_lHip.Position));
                    //shoulder_l2r.Normalize();hip_l2r.Normalize();
                    float angle = (float)Math.Acos(Math.Min(Vector3.Dot(shoulder_l2r, hip_l2r),1f));
                    torso.Rotation *= Quaternion.RotationAxis(new Vector3(0, 1, 0), angle);
                    torso.UpdateGrobalPose();
                    getRotation(neck, head, skeleton, sj_neck,
                        sj_head);
                    getRotation(r_shoulder, r_elbow, skeleton,
                        sj_rShoulder,sj_rElbow);
                    getRotation(r_elbow, r_hand, skeleton,
                        sj_rElbow,
                        sj_r_hand);
                    getRotation(l_shoulder, l_elbow, skeleton,
                        sj_lShoulder, sj_lElbow);
                    getRotation(l_elbow, l_hand, skeleton,
                        sj_lElbow, sj_l_hand);
                    getRotation(torso, neck, skeleton, sj_torso, sj_neck);
                    getRotation(r_hip, r_knee, skeleton,
                        sj_rHip,sj_rKnee);
                    getRotation(r_knee, r_foot, skeleton,
                       sj_rKnee, sj_rFoot);
                    getRotation(l_hip, l_knee, skeleton,
                       sj_lHip, sj_lKnee);
                    getRotation(l_knee, l_foot, skeleton,
                       sj_lKnee,sj_lFoot);

                }
            }
            return true;
        }

        private PMXBone getBone(string name)
        {
            foreach (var bone in bones)
            {
                if (bone.BoneName.Equals(name)) return bone;
            }
            return null;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="pos1">回転対象</param>
        /// <param name="pos2">目標ボーン</param>
        /// <param name="skeleton"></param>
        /// <param name="type1"></param>
        /// <param name="type2"></param>
        /// <returns></returns>
        private void getRotation(PMXBone b1, PMXBone b2,Skeleton skeleton, SkeletonJoint skel1,
            SkeletonJoint skel2)
        {

            SkeletonJoint targetBone = skel1;
            SkeletonJoint baseBone = skel2;
            if (targetBone.PositionConfidence < 0.5 || baseBone.PositionConfidence < 0.5) return;
            Vector3 t2b = Vector3.Normalize(ToVector3(targetBone.Position)-ToVector3(baseBone.Position));
            Vector3 it2b = Vector3.Normalize(Vector3.TransformCoordinate(b2.Position,b2.GlobalPose)-Vector3.TransformCoordinate(b1.Position,b1.GlobalPose));
            Vector3 axis = Vector3.Cross(t2b, it2b);
            float angle =(float) -Math.Acos(Vector3.Dot(t2b, it2b));
            b1.Rotation *= Quaternion.RotationAxis(axis, angle);
            b1.UpdateGrobalPose();
        }

        private Vector3 ToVector3(Point3D pos)
        {
            return new Vector3((float) pos.X,(float) -pos.Y,(float) -pos.Z);
        }
    }

    public class KinectUserLostEventArgs:EventArgs
    {
        public KinectUserLostEventArgs(UserData user)
        {
            data = user;
        }
        public UserData data;
    }

    public class KinectUserDetectEventArgs:EventArgs
    {
        public KinectUserDetectEventArgs(UserData user)
        {
            data = user;
        }
        public UserData data;
    }
}
