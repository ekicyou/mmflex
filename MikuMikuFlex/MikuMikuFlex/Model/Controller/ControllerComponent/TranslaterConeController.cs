using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMF.Matricies.Camera;
using MMF.Model.Shape.Overlay;
using SlimDX;

namespace MMF.Model.Controller.ControllerComponent
{
    class TranslaterConeController:OverlayConeShape
    {
        private DragControlManager dragController;

        public event EventHandler<TranslatedEventArgs> OnTranslated=delegate{}; 

        public TranslaterConeController(RenderContext context,ILockableController locker, Vector4 color, Vector4 overlayColor) : base(context, color, overlayColor)
        {
            dragController=new DragControlManager(locker);
        }

        public override void HitTestResult(bool result, bool mouseState, Point mousePosition)
        {
            base.HitTestResult(dragController.checkNeedHighlight(result), mouseState, mousePosition);
            dragController.checkBegin(result, mouseState, mousePosition);
            if (dragController.IsDragging)
            {
                checkTranslation();
            }
            dragController.checkEnd(result, mouseState, mousePosition);
        }

        private void checkTranslation()
        {
            Vector2 delta = dragController.Delta;
            CameraProvider cp = RenderContext.MatrixManager.ViewMatrixManager;
            Vector3 transformedAxis = Vector3.TransformNormal(Vector3.UnitY,//UnitY=(0,1,0)
                Transformer.LocalTransform * cp.ViewMatrix);//カメラから見たコーンの中心軸ベクトルを求める。
            //Transformer.Transformはモデルのローカル変形行列,
            //cp.ViewMatrixはカメラのビュー変換行列
            transformedAxis.Normalize();
            Vector3 cp2la = cp.CameraLookAt - cp.CameraPosition;//カメラの注視点からカメラの位置を引き目線のベクトルを求める
            //画面上での奥行きにあたるZベクトル
            cp2la.Normalize();//正規化

            Vector3 xUnit = Vector3.Cross(Vector3.UnitZ, Vector3.TransformNormal(cp.CameraUpVec, cp.ViewMatrix));//カメラの上方向ベクトルと目線のベクトルの外積を求め、
            //現在のカメラ位置における画面上のX軸方向が3DCG空間上で
            //どのベクトルで表されるか求める
            xUnit.Normalize();//正規化
            Vector3 yUnit = Vector3.Cross(xUnit, Vector3.UnitZ);//xUnitとcp2laにより画面上でのy軸が3DCG空間上でどのベクトルに
            //移されるのか求める。|xUnit|=|cp2la|=1のため、正規化は不要
            Vector3 deltaInDim3 = xUnit * delta.X + yUnit * delta.Y;//マウスの移動ベクトルを3CCG空間上で表すベクトルを求める。
            float dist = -Vector3.Dot(deltaInDim3, transformedAxis)/10f;
            OnTranslated(this,new TranslatedEventArgs(dist*Vector3.TransformNormal(Vector3.UnitY,Transformer.LocalTransform)));
        }

        public class TranslatedEventArgs : EventArgs
        {
            private Vector3 translation;

            public TranslatedEventArgs(Vector3 translation)
            {
                this.translation = translation;
            }

            public Vector3 Translation
            {
                get { return translation; }
            }
        }
    }
}
