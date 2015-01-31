using System;
using System.Collections.Generic;
using System.Diagnostics;
using MMF.Model;
using MMF.Morph;
using MMF.Utility;
using SlimDX;

namespace MMF.Bone
{
    /// <summary>
    ///  CCDIKでIKボーンを更新するクラス
    /// </summary>
    public class CCDIK : ITransformUpdater
    {
        /// <summary>
        /// IKボーンのリスト
        /// </summary>
        private List<PMXBone> IKbones;

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="IKbones">IKボーンのリスト</param>
        public CCDIK(List<PMXBone> IKbones)
        {
            this.IKbones = IKbones;
        }

        /// <summary>
        ///  ITransformUpdaterメンバーの実装
        /// </summary>
        public bool UpdateTransform()
        {
            foreach (PMXBone IKbone in IKbones) UpdateEachIKBoneTransform(IKbone);
            return false;
        }

        /// <summary>
        /// 各IKボーンのアップデート
        /// </summary>
        private void UpdateEachIKBoneTransform(PMXBone IKbone)
        {
            // ループ回数のリセット
            foreach (var link in IKbone.ikLinks) link.loopCount = 0;

            // 決められた回数IKループを繰り返す
            for (int it = 0; it < IKbone.Iterator; it++) IKloop(IKbone);
        }

        /// <summary>
        /// IK計算のループ
        /// </summary>
        /// <param name="IKbone">IKボーン</param>
        private void IKloop(PMXBone IKbone)
        {
            var effector = IKbone.IkTargetBone;
            var TargetGlobalPos = Vector3.TransformCoordinate(IKbone.Position, IKbone.GlobalPose);
            foreach (var ikLink in IKbone.ikLinks)
            {
                var link2Effector = GetLink2Effector(ikLink, effector);
                var link2Target = GetLink2Target(ikLink, TargetGlobalPos);
                IKLinkCalc(ikLink, link2Effector, link2Target, IKbone.RotationLimited);
            }
        }


        /// <summary>
        /// IKリンク基準のエフェクタ一位置を取得する
        /// </summary>
        /// <param name="ikLink">IKリンク</param>
        /// <param name="effector">エフェクタ</param>
        /// <returns>IKリンク基準のエフェクタ一位置</returns>
        private Vector3 GetLink2Effector(IkLink ikLink, PMXBone effector)
        {
            var ToLinkLocal = Matrix.Invert(ikLink.ikLinkBone.GlobalPose);
            var effectorPos = Vector3.TransformCoordinate(effector.Position, effector.GlobalPose * ToLinkLocal); //●
            return Vector3.Normalize(effectorPos - ikLink.ikLinkBone.Position);
        }

        /// <summary>
        /// IKリンク基準のターゲット位置を取得する
        /// </summary>
        /// <param name="ikLink">IKリンク</param>
        /// <param name="TargetGlobalPos">グローバル基準ターゲット位置</param>
        /// <returns>IKリンク基準のターゲット位置</returns>
        private Vector3 GetLink2Target(IkLink ikLink, Vector3 TargetGlobalPos)
        {
            var ToLinkLocal = Matrix.Invert(ikLink.ikLinkBone.GlobalPose);
            Vector3 targetPos;
            Vector3.TransformCoordinate(ref TargetGlobalPos, ref ToLinkLocal, out targetPos);
            return Vector3.Normalize(targetPos - ikLink.ikLinkBone.Position);
        }

        /*
         *CCD-IKのIKLink計算の仕組み
         *-◎-◎-◎-●
         * ↑関節      ▲←IKターゲット(MMD内ではIKBoneと呼ぶ)。関節を近づけたい目標
         * (関節内の◎にあたる、IKで曲げる途中の関節をIKリンク、●にあたる関節の終点をエフェクタ(MMD内ではターゲットボーン)
         * とする。
         * 1)まず、◎-●の方向ベクタを求める。ベクトルA
         * 2)次に、◎-▲の方向ベクタを求める。ベクトルB
         * 3)∠●◎▲における最短回転角度を求める。arccos(A・B)よりラジアン単位で求まる
         * 4)２つの方向ベクトルの外積から回転軸を求める
         */
        /// <summary>
        /// IKLinkを計算
        /// </summary>
        /// <param name="ikLink">IKリンク</param>
        /// <param name="link2Effector">IKリンク基準のエフェクタ位置</param>
        /// <param name="link2Target">IKリンク基準のターゲット位置</param>
        /// <param name="RotationLimited">回転角度の上下限値の絶対値</param>
        private void IKLinkCalc(IkLink ikLink, Vector3 link2Effector, Vector3 link2Target, float RotationLimited)
        {
            //回転角度を求める
            var dot = Vector3.Dot(link2Effector, link2Target);
            if (dot > 1f) dot = 1f;
            var rotationAngle = ClampFloat((float)Math.Acos(dot), RotationLimited);
            if (float.IsNaN(rotationAngle)) return;
            if (rotationAngle <= 1.0e-3f) return;

            //回転軸を求める
            var rotationAxis = Vector3.Cross(link2Effector, link2Target);
            ikLink.loopCount++;

            //軸を中心として回転する行列を作成する。
            var rotation = Quaternion.RotationAxis(rotationAxis, rotationAngle);
            rotation.Normalize();
            ikLink.ikLinkBone.Rotation = rotation * ikLink.ikLinkBone.Rotation;

            #region 回転量制限
            RestrictRotation(ikLink);
            #endregion

            ikLink.ikLinkBone.UpdateGrobalPose();
        }

        /// <summary>
        /// 数をある上下限の絶対値でクランプする
        /// </summary>
        /// <param name="f">クランプされる数</param>
        /// <param name="limit">上下限値の絶対値</param>
        /// <returns>クランプされた数</returns>
        private float ClampFloat(float f, float limit)
        {
            return Math.Max(Math.Min(f, limit), -limit);
        }

        /// <summary>
        /// 回転量を制限する
        /// </summary>
        /// <param name="ikLink">IKリンク</param>
        private void RestrictRotation(IkLink ikLink)
        {
            if (!ikLink.isLimited) return;

            float xRotation, yRotation, zRotation;
            var type = SplitRotation(ikLink.ikLinkBone.Rotation, out xRotation, out yRotation, out zRotation);
            var clamped = Vector3.Clamp(new Vector3(xRotation, yRotation, zRotation).NormalizeEular(), ikLink.minRot, ikLink.maxRot);
            xRotation = clamped.X;
            yRotation = clamped.Y;
            zRotation = clamped.Z;
            switch (type)
            {
                case 0:
                    ikLink.ikLinkBone.Rotation = Quaternion.RotationMatrix(Matrix.RotationX(xRotation) * Matrix.RotationY(yRotation) * Matrix.RotationZ(zRotation));
                    break;
                case 1:
                    ikLink.ikLinkBone.Rotation = Quaternion.RotationMatrix(Matrix.RotationY(yRotation) * Matrix.RotationZ(zRotation) * Matrix.RotationX(xRotation));
                    break;
                case 2:
                    ikLink.ikLinkBone.Rotation = Quaternion.RotationYawPitchRoll(yRotation, xRotation, zRotation);
                    break;
            }
        }

        /// <summary>
        /// //X,Y,Z軸回転量に分解する
        /// </summary>
        /// <param name="Rotation">回転量</param>
        /// <param name="xRotation">X軸回転量</param>
        /// <param name="yRotation">Y軸回転量</param>
        /// <param name="zRotation">Z軸回転量</param>
        /// <returns>分解パターン</returns>
        private int SplitRotation(Quaternion Rotation, out float xRotation, out float yRotation, out float zRotation)
        {
            if (CGHelper.FactoringQuaternionXYZ(Rotation, out xRotation, out yRotation, out zRotation)) return 0;
            if (CGHelper.FactoringQuaternionYZX(Rotation, out yRotation, out zRotation, out xRotation)) return 1;
            CGHelper.FactoringQuaternionZXY(Rotation, out zRotation, out xRotation, out yRotation);
            return 2;
        }


    }
}