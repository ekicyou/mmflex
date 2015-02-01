using System;
using System.IO;
using Assimp;
using MMDFileParser.PMXModelParser;
using MMF.Model;
using MMF.Model.Assimp;
using MMF.Utility;
using SlimDX;
using SlimDX.Direct3D11;

namespace MMF.MME.VariableSubscriber.MaterialSubscriber
{
    public class MaterialInfo : IDisposable
    {
        //TODO 対応しているマテリアルが完全ではない
        public Vector4 AmbientColor;
        public Vector4 DiffuseColor;
        public Vector4 EdgeColor;
        public Vector4 EmissiveColor;
        public Vector4 GroundShadowColor;
        public bool IsOperationAdd;
        public bool IsSubtextureUsed;
        public bool IsToonUsed;
        public ShaderResourceView MaterialSphereMap;
        public ShaderResourceView MaterialTexture;
        public Vector4 SpecularColor;
        public float SpecularPower;
        public Vector4 SphereAddValue;
        public SphereMode SphereMode;
        public Vector4 SphereMulValue;
        public Vector4 TextureAddValue;
        public Vector4 TextureMulValue;
        public Vector4 ToonColor;
        public ShaderResourceView ToonTexture;
        public float EdgeSize;
        public bool isEdgeEnable;
        public bool isGroundShadowEnable;

        /// <summary>
        /// 材質モーフ用、乗算値保持メンバ
        /// </summary>
        public  MaterialInfo MulMaterialInfo;

        /// <summary>
        /// 材質モーフ用、乗算値保持メンバ
        /// </summary>
        public MaterialInfo AddMaterialInfo;

        /// <summary>
        /// 材質モーフ用、乗算値保持メンバ
        /// </summary>
        public MaterialInfo InitialMaterialInfo;

        public void Dispose()
        {
            if (MulMaterialInfo != null) MulMaterialInfo.Dispose();
            if (AddMaterialInfo != null) AddMaterialInfo.Dispose();
            if (MaterialTexture != null && !MaterialTexture.Disposed) MaterialTexture.Dispose();
            if (MaterialSphereMap != null && !MaterialSphereMap.Disposed) MaterialSphereMap.Dispose();
            if (ToonTexture != null && !ToonTexture.Disposed) ToonTexture.Dispose();
        }

        public void UpdateMaterials()
        {
            AmbientColor = CGHelper.MulEachMember(InitialMaterialInfo.AmbientColor,MulMaterialInfo.AmbientColor) +AddMaterialInfo.AmbientColor;
            DiffuseColor = CGHelper.MulEachMember(InitialMaterialInfo.DiffuseColor, MulMaterialInfo.DiffuseColor) +
                           AddMaterialInfo.DiffuseColor;
            SpecularColor = CGHelper.MulEachMember(InitialMaterialInfo.SpecularColor, MulMaterialInfo.SpecularColor) +
                            AddMaterialInfo.SpecularColor;
            SpecularPower = InitialMaterialInfo.SpecularPower*MulMaterialInfo.SpecularPower +
                            AddMaterialInfo.SpecularPower;
            EdgeColor = CGHelper.MulEachMember(InitialMaterialInfo.EdgeColor, MulMaterialInfo.EdgeColor) +
                        AddMaterialInfo.EdgeColor;
            ResetMorphMember();
        }

        private void ResetMorphMember()
        {
            MulMaterialInfo.AmbientColor=new Vector4(1f);
            MulMaterialInfo.DiffuseColor=new Vector4(1f);
            MulMaterialInfo.SpecularColor=new Vector4(1f);
            MulMaterialInfo.SpecularPower = 1f;
            MulMaterialInfo.EdgeColor = new Vector4(0f);
            AddMaterialInfo.AmbientColor = new Vector4(0f);
            AddMaterialInfo.DiffuseColor = new Vector4(0f);
            AddMaterialInfo.SpecularColor = new Vector4(0f);
            AddMaterialInfo.SpecularPower = 0f;
            AddMaterialInfo.EdgeColor = new Vector4(0f);
        }

        /// <summary>
        ///     マテリアル情報から作成
        /// </summary>
        /// <param name="drawable">適用対象のIDrawable</param>
        /// <param name="data">マテリアル情報</param>
        /// <returns>MMEエフェクト用マテリアル情報</returns>
        public static MaterialInfo FromMaterialData(IDrawable drawable, MaterialData data)
        {
            MaterialInfo info = new MaterialInfo();
            info.AmbientColor = new Vector4(data.Ambient, 1f);
            info.DiffuseColor = data.Diffuse;
            info.EdgeColor = data.EdgeColor;
            info.EmissiveColor = new Vector4(1f);
            info.GroundShadowColor = drawable.GroundShadowColor;
            info.SpecularColor = new Vector4(data.Specular, 1f);
            info.SpecularPower = data.SpecularCoefficient;
            info.ToonColor = new Vector4(0f);
            info.EdgeSize = data.EdgeSize;
            info.isEdgeEnable = data.bitFlag.HasFlag(RenderFlag.RenderEdge);
            info.isGroundShadowEnable = data.bitFlag.HasFlag(RenderFlag.GroundShadow);
            info.MulMaterialInfo=new MaterialInfo();
            info.AddMaterialInfo=new MaterialInfo();
            MaterialInfo initialInfo=new MaterialInfo();
            initialInfo.AmbientColor = new Vector4(data.Ambient, 1f);
            initialInfo.DiffuseColor = data.Diffuse;
            initialInfo.EdgeColor = data.EdgeColor;
            initialInfo.EmissiveColor = new Vector4(1f);
            initialInfo.GroundShadowColor = drawable.GroundShadowColor;
            initialInfo.SpecularColor = new Vector4(data.Specular, 1f);
            initialInfo.SpecularPower = data.SpecularCoefficient;
            initialInfo.ToonColor = new Vector4(0f);
            initialInfo.EdgeSize = data.EdgeSize;
            info.InitialMaterialInfo = initialInfo;
            info.ResetMorphMember();
            return info;
        }

        public static MaterialInfo FromMaterialData(IDrawable drawable, Material material,RenderContext context,ISubresourceLoader loader)
        {
            MaterialInfo info=new MaterialInfo();
            info.AmbientColor = material.ColorAmbient.ToSlimDX();
            if (info.AmbientColor == Vector4.Zero)
            {
                info.AmbientColor=new Vector4(1f,1f,1f,1f);
            }
            info.DiffuseColor = material.ColorDiffuse.ToSlimDX();
            if (info.DiffuseColor == Vector4.Zero)
            {
                info.DiffuseColor=new Vector4(1f,1f,1f,1f);
            }
            else if(info.DiffuseColor.W==0f)
            {
                info.DiffuseColor.W = 1f;
            }
            info.EmissiveColor = material.ColorEmissive.ToSlimDX();
            info.SpecularColor = material.ColorSpecular.ToSlimDX();
            info.SpecularPower = material.ShininessStrength;
            if (info.SpecularColor == Vector4.Zero)
            {
                info.SpecularColor=new Vector4(0.1f);
            }
            
            info.GroundShadowColor = drawable.GroundShadowColor;
            info.isEdgeEnable = false;
            info.isGroundShadowEnable = true;
            info.SphereMode=SphereMode.Disable;
            info.IsToonUsed = false;
            if (material.GetMaterialTextures(TextureType.Diffuse) != null)
            {
                Stream stream = loader.getSubresourceByName(material.GetMaterialTextures(TextureType.Diffuse)[0].FilePath);
                if(stream!=null)
                using (
                    Texture2D texture = Texture2D.FromStream(context.DeviceManager.Device, stream, (int) stream.Length))
                {
                    info.MaterialTexture = new ShaderResourceView(context.DeviceManager.Device, texture);
                }
                stream.Close();
            }
            return info;
        }
    }
}