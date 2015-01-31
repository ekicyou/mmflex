using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using MMDFileParser.MotionParser;
using MMF.Matricies.Projection;
using MMF.Utility;
using SlimDX;

namespace MMF.Matricies.Camera.CameraMotion
{
    /// <summary>
    /// VMDファイルを用いて動かすカメラ用のモーションを管理するクラス
    /// This class manage camera moved with vmd moton file.
    /// </summary>
    public class VMDCameraMotionProvider:ICameraMotionProvider
    {
        public static VMDCameraMotionProvider OpenFile(string path)
        {
            return new VMDCameraMotionProvider(MotionData.getMotion(File.OpenRead(path)));
        }

        private List<CameraFrameData> CameraFrames;

        public float CurrentFrame
        {
            get { return currentFrame; }
            set { currentFrame = value; }
        }

        private Stopwatch stopWatch;
        private long lastMillisecound;
        private float currentFrame=0;
        private bool isPlaying = false;
        private float finalFrame;
        private bool needReplay;

        /// <summary>
        /// コンストラクタ
        /// モーションを再生する際は、インスタンス作成の後Startメソッドを呼ぶこと
        /// Constractor
        /// If you want to play camera motion,you should call Start method after making instance.
        /// </summary>
        /// <param name="cameraMotion">VMDファイルのデータ(VMD parsed data)</param>
        public VMDCameraMotionProvider(MotionData cameraMotion)
        {
            CameraFrames = cameraMotion.CameraFrames.CameraFrames;
            CameraFrames.Sort(new CameraFrameData());
            stopWatch=new Stopwatch();
            if (CameraFrames.Count == 0) finalFrame = 0;
            else
            finalFrame = CameraFrames.Last().FrameNumber;
        }

        public float FinalFrame
        {
            get { return finalFrame; }
        }

        /// <summary>
        /// モーションを再生する
        /// Start to play motion
        /// </summary>
        public void Start(float startFrame=0,bool needReplay=false)
        {
            stopWatch.Start();
            currentFrame = startFrame;
            isPlaying = true;
            this.needReplay = needReplay;
        }

        /// <summary>
        /// モーションを停止する
        /// Stop to play motion
        /// </summary>
        public void Stop()
        {
            stopWatch.Stop();
            isPlaying = false;
        }

        

        private void Leap(CameraProvider cp,IProjectionMatrixProvider projection,float frame)
        {
            if(CameraFrames.Count==0)return;
            for (int j = 0; j < CameraFrames.Count - 1; j++)
            {
                if (CameraFrames[j].FrameNumber < frame && CameraFrames[j + 1].FrameNumber >= frame)
                {
                    //フレームが挟まれている時
                    
                    uint frameMargin = CameraFrames[j + 1].FrameNumber - CameraFrames[j].FrameNumber;
                    float progress = (frame - CameraFrames[j].FrameNumber)/(float) frameMargin;
                    LeapFrame(CameraFrames[j], CameraFrames[j + 1], cp, projection,progress);
                    return;
                }
            }
            //returnされなかったとき(つまり最終フレーム以降のとき)
            LeapFrame(CameraFrames.Last(),CameraFrames.Last(),cp,projection,0);
        }

        private void LeapFrame(CameraFrameData cf1,CameraFrameData cf2,CameraProvider cp,IProjectionMatrixProvider proj,float f)
        {
            float ProgX, ProgY, ProgZ, ProgR, ProgL, ProgP; ;
            ProgX = cf1.Curves[0].Evaluate(f);
            ProgY = cf1.Curves[1].Evaluate(f);
            ProgZ = cf1.Curves[2].Evaluate(f);
            ProgR = cf1.Curves[3].Evaluate(f);
            ProgL = cf1.Curves[4].Evaluate(f);
            ProgP = cf1.Curves[5].Evaluate(f);
            cp.CameraLookAt = CGHelper.ComplementTranslate(cf1, cf2,
                new Vector3(ProgX, ProgY, ProgZ));
            Quaternion rotation = CGHelper.ComplementRotateQuaternion(cf1,cf2,
                ProgR);
            float length = CGHelper.Lerp(cf1.Distance,cf2.Distance, ProgL);
            float angle = CGHelper.Lerp(cf1.ViewAngle,cf2.ViewAngle, ProgP);
            Vector3 Position2target = Vector3.TransformCoordinate(new Vector3(0, 0, 1),
                Matrix.RotationQuaternion(rotation));
            Vector3 TargetPosition = cp.CameraLookAt + length * Position2target;
            cp.CameraPosition = TargetPosition;
            proj.Fovy = CGHelper.ToRadians(angle);
        }

        public void UpdateCamera(CameraProvider cp, IProjectionMatrixProvider proj)
        {
            if (lastMillisecound == 0)
            {
                lastMillisecound = stopWatch.ElapsedMilliseconds;
            }
            else
            {
                long currentMillisecound = stopWatch.ElapsedMilliseconds;
                long elapsed = currentMillisecound - lastMillisecound;//前回とのフレームの差
                if (isPlaying) currentFrame += elapsed/30f;
                if (needReplay && finalFrame < currentFrame) currentFrame = 0;
                lastMillisecound = currentMillisecound;
            }
            Leap(cp, proj, currentFrame);
        }
    }
}
