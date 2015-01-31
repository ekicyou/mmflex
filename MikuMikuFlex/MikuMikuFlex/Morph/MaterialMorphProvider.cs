using System.Collections.Generic;
using MMDFileParser.PMXModelParser;
using MMF.MME.VariableSubscriber.MaterialSubscriber;
using MMF.Model;
using MMF.Model.PMX;
using MMF.Motion;
using SlimDX;

namespace MMF.Morph
{
    class MaterialMorphProvider:IMorphProvider
    {
        public Dictionary<string,MaterialMorphData> Morphs=new Dictionary<string, MaterialMorphData>();

        private PMXModel model;

        public MaterialMorphProvider(PMXModel model)
        {
            this.model = model;
            foreach (MorphData materialMorphData in model.Model.MorphList.Morphes)
            {
             if(materialMorphData.type==MorphType.Matrial)Morphs.Add(materialMorphData.MorphName,new MaterialMorphData(materialMorphData));
            }
        }

        public void ApplyMorphProgress(float frameNumber, IEnumerable<MorphMotion> morphMotions)
        {
            foreach (var morphMotion in morphMotions)
            {
                SetMorphProgress(morphMotion.GetMorphValue(frameNumber), morphMotion.MorphName);
            }
        }

        public bool ApplyMorphProgress(float progress, string morphName)
        {
            return SetMorphProgress(progress, morphName);
        }

        public void UpdateFrame()
        {
            
        }

        private bool SetMorphProgress(float progress, string morphName)
        {
            if (!Morphs.ContainsKey(morphName)) return false;
            MaterialMorphData data = Morphs[morphName];
            foreach (var materialMorphOffset in data.Morphoffsets)
            {
                if (materialMorphOffset.MaterialIndex == -1)
                {
                    foreach (var pmxSubset in model.SubsetManager.Subsets)
                    {
                        MaterialInfo matInfo = pmxSubset.MaterialInfo;
                        matInfo = materialMorphOffset.OffsetCalclationType == 0 ? matInfo.MulMaterialInfo : matInfo.AddMaterialInfo;//0の場合は対象を乗算、1なら対象を加算にセット
                        matInfo.DiffuseColor += materialMorphOffset.Diffuse*progress;
                        matInfo.AmbientColor += new Vector4(materialMorphOffset.Ambient, 1f) * progress;
                        matInfo.SpecularColor += new Vector4(materialMorphOffset.Specular, 1f) * progress;
                        matInfo.SpecularPower += materialMorphOffset.SpecularCoefficient * progress;
                        matInfo.EdgeColor += materialMorphOffset.EdgeColor * progress;
                    }
                }
                else
                {
                    MaterialInfo matInfo = model.SubsetManager.Subsets[materialMorphOffset.MaterialIndex].MaterialInfo;
                    matInfo=materialMorphOffset.OffsetCalclationType==0?matInfo.MulMaterialInfo:matInfo.AddMaterialInfo;//0の場合は対象を乗算、1なら対象を加算にセット
                    matInfo.DiffuseColor += materialMorphOffset.Diffuse * progress;
                    matInfo.AmbientColor += new Vector4(materialMorphOffset.Ambient, 1f) * progress;
                    matInfo.SpecularColor += new Vector4(materialMorphOffset.Specular, 1f) * progress;
                    matInfo.SpecularPower += materialMorphOffset.SpecularCoefficient * progress;
                    matInfo.EdgeColor += materialMorphOffset.EdgeColor * progress;
                }
            }
            return true;
        }
    }
}
