using System.Collections.Generic;
using System.IO;
using System.Linq;
using MMDFileParser.PMXModelParser;
using MMF.MME;
using SlimDX.Direct3D11;

namespace MMF.Model.PMX
{
    /// <summary>
    /// </summary>
    internal class PMXSubsetManager : ISubsetManager
    {
        private readonly ModelData model;
        // private readonly BasicMaterialConstantBufferManager basicMaterialConstantBuffer = new BasicMaterialConstantBufferManager();
        private Device device;
        //private BasicMaterialConstantBufferInputLayout materialBuffer;
        private ISubresourceLoader subresourceManager;
        private IToonTextureManager toonManager;
        public List<ISubset> Subsets { get; private set; }

        public PMXSubsetManager(PMXModel drawable,ModelData model)
        {
            this.model = model;
            this.Drawable = drawable;

        }

        public void Initialze(RenderContext context, MMEEffectManager effectManager, ISubresourceLoader subresourceManager,
            IToonTextureManager ToonManager)
        {
            MMEEffect = effectManager;
            toonManager = ToonManager;
            Subsets = new List<ISubset>();
            this.device = context.DeviceManager.Device;
            this.subresourceManager = subresourceManager;
            ModelData model = this.model;
            int vertexSum = 0;
            for (int i = 0; i < model.MaterialList.MaterialCount; i++)
            {
                MaterialData material = model.MaterialList.Materials[i];//サブセットデータを作成する
                PMXSubset dr = new PMXSubset(Drawable,material,i);
                dr.DoCulling = !material.bitFlag.HasFlag(RenderFlag.CullNone);
                dr.VertexCount = material.VertexNumber/3;
                dr.StartIndex = vertexSum;
                if (material.textureIndex >= toonManager.ResourceViews.Length)
                {
                    if (toonManager.ResourceViews.Length == 0)
                    {
                        int index = ToonManager.LoadToon(model.TextureList.TexturePathes[material.textureIndex]);
                        dr.MaterialInfo.ToonTexture = ToonManager.ResourceViews[index];
                        dr.MaterialInfo.IsToonUsed = false;
                    }
                    else
                    {
                        dr.MaterialInfo.ToonTexture = ToonManager.ResourceViews[0];
                        dr.MaterialInfo.IsToonUsed = false;
                    }
                }
                else
                {
                    if (material.ShareToonFlag == 1)
                    {
                        dr.MaterialInfo.ToonTexture = ToonManager.ResourceViews[material.textureIndex + 1];
                        dr.MaterialInfo.IsToonUsed = true;
                    }
                    else if (material.textureIndex != -1)
                    {
                        if (model.TextureList.TexturePathes.Count < material.textureIndex + 1)
                        {
                            dr.MaterialInfo.ToonTexture = ToonManager.ResourceViews[0];
                            dr.MaterialInfo.IsToonUsed = true;
                        }
                        else
                        {
                            int index = ToonManager.LoadToon(model.TextureList.TexturePathes[material.textureIndex]);
                            dr.MaterialInfo.ToonTexture = ToonManager.ResourceViews[index];
                            dr.MaterialInfo.IsToonUsed = true;
                        }
                    }
                    else
                    {
                        dr.MaterialInfo.ToonTexture = ToonManager.ResourceViews[0];
                        dr.MaterialInfo.IsToonUsed = true;
                    }
                }
                vertexSum += material.VertexNumber;

                //テクスチャの読み込み
                if (material.TextureTableReferenceIndex != -1)
                {
                    dr.MaterialInfo.MaterialTexture = getSubresourceById(model.TextureList.TexturePathes[material.TextureTableReferenceIndex]);
                }
                //スフィアマップの読み込み
                if (material.SphereTextureTableReferenceIndex != -1)
                {
                    dr.MaterialInfo.SphereMode = material.SphereMode;
                    dr.MaterialInfo.MaterialSphereMap =
                        getSubresourceById(model.TextureList.TexturePathes[material.SphereTextureTableReferenceIndex]);
                }
                Subsets.Add(dr);
            }
        }

        public void ResetEffect(MMEEffectManager effect)
        {
            this.MMEEffect=effect;
        }


        public void DrawAll()
        {
            for (int i = 0; i < Subsets.Count; i++)
            {
                UpdateConstantByMaterial(Subsets[i]);
                MMEEffect.ApplyEffectPass(Subsets[i], MMEEffectPassType.Object, (subset) => subset.Draw(device));

            }
        }

        public void DrawEdges()
        {
            //TODO エッジ描画の実装
            //foreach (PMXSubset variable in from subset in Subsets where  subset.MaterialInfo.isEdgeEnable select subset)
            //{
            //    UpdateConstantByMaterial(variable);
            //    MMEEffect.ApplyEffectPass(variable, MMEEffectPassType.Edge, (subset) => subset.Draw(device));
            //}
        }

        public void DrawGroundShadow()
        {
            //TODO 地面陰の実装
            //foreach (PMXSubset variable in from subset in Subsets where subset.MaterialInfo.isGroundShadowEnable select subset)
            //{
            //    UpdateConstantByMaterial(variable);
            //    MMEEffect.ApplyEffectPass(variable, MMEEffectPassType.Shadow, (subset) => subset.Draw(device));
            //}
        }

        public int SubsetCount
        {
            get
            {
                return Subsets.Count;
            }
        }

        public void Dispose()
        {
            foreach (PMXSubset drawableResource in Subsets)
            {
                drawableResource.Dispose();
            }
        }


        private void UpdateConstantByMaterial(ISubset ipmxSubset)
        {
            MMEEffect.ApplyAllMaterialVariables(ipmxSubset.MaterialInfo);
        }

        private ShaderResourceView getSubresourceById(string p)
        {
            using (Stream stream = subresourceManager.getSubresourceByName(p))
            {
                if (stream == null) return null;
                return ShaderResourceView.FromStream(device, stream, (int) stream.Length);
            }
        }

        public IDrawable Drawable { get; set; }

        public MMEEffectManager MMEEffect { get; set; }
    }
}