using System;
using System.Collections.Generic;
using System.Linq;
using NiTEWrapper;
using OpenNIWrapper;

namespace MMF.Kinect
{
    /// <summary>
    /// Kinectのデバイスを管理するクラス
    /// </summary>
    public class KinectDeviceManager:IDisposable
    {
        private UserTracker _niteUserTracker;

        /// <summary>
        /// 現在のユーザートラッカーのフレーム
        /// </summary>
        public UserTrackerFrameRef CurrentUserTrackerFrameRef
        {
            get
            {
                if (hasNewValue)
                {
                    _currentUserTrackerFrameRef = _niteUserTracker.readFrame();
                    UpdateUserData();
                    hasNewValue = false;
                }
                return _currentUserTrackerFrameRef;
            }
        }

        private void UpdateUserData()
        {
            //トラッキング済みのユーザーの配列を作成する
            TrackedUsers =( from user in _currentUserTrackerFrameRef.Users
                where user.Skeleton.State == Skeleton.SkeletonState.TRACKED
                select user).ToDictionary((user)=>user.UserId,(user)=>user);
        }

        public Device KinnectDevice { get; private set; }

        private bool hasNewValue=true;

        private UserTrackerFrameRef _currentUserTrackerFrameRef;

        public UserTracker NiteUserTracker
        {
            get
            {
                return _niteUserTracker;
            }
 
        }

        public Dictionary<short,UserData> TrackedUsers;

        private short _userCursor;

        public short UserCursor
        {
            get { return _userCursor; }
            set
            {
                _userCursor = value;
                if (!_currentUserTrackerFrameRef.getUserById(value).isValid)
                    _userCursor = 0;
            }
        }

        public void StartTracking()
        {
            NiteUserTracker.StartSkeletonTracking(_userCursor);
        }

        void _niteUserTracker_onNewData(UserTracker uTracker)
        {
            hasNewValue = true;
        }

        internal KinectDeviceManager(Device device)
        {
            KinnectDevice = device;
            _niteUserTracker = UserTracker.Create(KinnectDevice);
            _niteUserTracker.onNewData += _niteUserTracker_onNewData;
        }

        public void Dispose()
        {
            KinnectDevice.Close();
            if(NiteUserTracker!=null)NiteUserTracker.Destroy();
        }
    }
}
