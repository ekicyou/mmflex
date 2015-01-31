using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMF.Matricies.Camera;
using MMF.Matricies.Projection;
using SlimDX;

namespace _05_HowToUpdateCamera
{
    //①-C 独自定義のカメラモーションを記述してみる
    class SimpleUserDefinitionCameraMotionProvider:ICameraMotionProvider
    {
        private float t;

        public void UpdateCamera(CameraProvider cp, IProjectionMatrixProvider proj)
        {
            t += 0.01f;
            cp.CameraPosition =new Vector3(0,0,(float)Math.Sin(t)*50);//tに応じて三角関数によりカメラの位置を変更する
            /*
             * 説明：
             * CameraProvider.CameraPositionがカメラの位置を指す
             * CameraProvider.CameraLookAtがカメラの注視点を指す
             * CameraProvider.CameraUpVecがカメラの上方向ベクトルを指す
             * 
             * IProjectionMatrixProviderのプロパティは以下
             * ・ZNear 近クリップ距離
             * ・ZFar 遠クリップ距離
             * ・AspectRatio アスペクト比
             * ・Fovy 視野角
             */
        }
    }
}
