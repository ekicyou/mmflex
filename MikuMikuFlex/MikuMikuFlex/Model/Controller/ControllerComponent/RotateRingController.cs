using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMF.Matricies.Camera;
using MMF.Model.Shape.Overlay;
using SlimDX;
using SlimDX.DirectInput;

namespace MMF.Model.Controller.ControllerComponent
{
    class RotateRingController:OverlaySilinderShape
    {
        private readonly DragControlManager dragController;


        public EventHandler<RotationChangedEventArgs> OnRotated = delegate { }; 

        private bool lastState;

        private bool lastMouseState;

        private bool isDragging;

        private Point lastPoint;

        public RotateRingController(RenderContext context,ILockableController parent, Vector4 color, Vector4 overlayColor, SilinderShapeDescription desc) : base(context, color, overlayColor, desc)
        {
            dragController=new DragControlManager(parent);
        }

        public override void HitTestResult(bool result, bool mouseState, Point mousePosition)
        {

            base.HitTestResult(dragController.checkNeedHighlight(result), mouseState, mousePosition);
            dragController.checkBegin(result,mouseState,mousePosition);
            if (dragController.IsDragging)
            {
                float t = calculateLength(dragController.Delta);
                var a = Vector3.TransformNormal(Vector3.UnitY, Transformer.LocalTransform);
                a.Normalize();
                OnRotated(this,new RotationChangedEventArgs(a,t));
            }
            dragController.checkEnd(result,mouseState,mousePosition);
        }

        private float calculateLength(Vector2 delta)//deltaはマウスの画面上の座標の偏移量である
        {
            CameraProvider cp = RenderContext.MatrixManager.ViewMatrixManager;
            Vector3 transformedAxis = Vector3.TransformNormal(Vector3.UnitY,//UnitY=(0,1,0)
                Transformer.LocalTransform*cp.ViewMatrix);//カメラから見たシリンダの中心軸ベクトルを求める。
                                                     //Transformer.Transformはモデルのローカル変形行列,
                                                     //cp.ViewMatrixはカメラのビュー変換行列
            Vector3 cp2la =cp.CameraLookAt-cp.CameraPosition;//カメラの注視点からカメラの位置を引き目線のベクトルを求める
            //画面上での奥行きにあたるZベクトル
            cp2la.Normalize();//正規化
            Vector3 transformUnit = Vector3.Cross(Vector3.UnitZ, transformedAxis);
            //カメラから見ているので(0,0,1)とシリンダの中心軸ベクトルの外積によって求まるベクトルが
            //このシリンダにとっての値を上下するときの方向ベクトルとして求まる。
            transformUnit.Normalize(); //正規化

            Vector3 xUnit = Vector3.Cross(Vector3.UnitZ, Vector3.TransformNormal(cp.CameraUpVec,cp.ViewMatrix));//カメラの上方向ベクトルと目線のベクトルの外積を求め、
                                                                 //現在のカメラ位置における画面上のX軸方向が3DCG空間上で
                                                                 //どのベクトルで表されるか求める
            xUnit.Normalize();//正規化
            Vector3 yUnit = Vector3.Cross(xUnit, Vector3.UnitZ);//xUnitとcp2laにより画面上でのy軸が3DCG空間上でどのベクトルに
                                                        //移されるのか求める。|xUnit|=|cp2la|=1のため、正規化は不要
            Vector3 deltaInDim3 = xUnit*delta.X + yUnit*delta.Y;//マウスの移動ベクトルを3CCG空間上で表すベクトルを求める。
            return Vector3.Dot(deltaInDim3, transformUnit);//マウスの移動ベクトルとシリンダの値の上下のための方向ベクトルに
                                                          //どの程度含まれてるか求めるため内積を求め、これを偏移量の基準とする。
        }

        public class RotationChangedEventArgs:EventArgs
        {
            private float length;

            private Vector3 axis;

            public RotationChangedEventArgs(Vector3 axis, float length)
            {
                this.axis = axis;
                this.length = length;
            }

            public Vector3 Axis
            {
                get { return axis; }
            }

            public float Length
            {
                get { return length; }
            }
        }
    }
}
