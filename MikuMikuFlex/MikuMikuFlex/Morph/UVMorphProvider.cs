using System;
using System.Collections.Generic;
using MMDFileParser.PMXModelParser;
using MMDFileParser.PMXModelParser.MorphOffset;
using MMF.Model;
using MMF.Model.PMX;
using MMF.Motion;
using SlimDX;

namespace MMF.Morph
{
    public class UVMorphProvider:IMorphProvider
    {
        public Dictionary<string,UVMorphData> Morphs=new Dictionary<string, UVMorphData>();

        private ModelData model;

        private IBufferManager bufferManager;

        private MorphType targetMorph;

        public UVMorphProvider(PMXModel model,MorphType targetType)
        {
            bufferManager = model.BufferManager;
            targetMorph = targetType;
            this.model = model.Model;
            if(model.Model.Header.AdditionalUVCount+2<=(int)targetType)return;//このとき対応した追加UVは存在しない
            foreach (MorphData morphData in model.Model.MorphList.Morphes)
            {
                if (morphData.type == targetMorph)
                {
                    Morphs.Add(morphData.MorphName,new UVMorphData(morphData));
                }
            }
        }

        public void ApplyMorphProgress(float frameNumber, IEnumerable<MorphMotion> morphMotions)
        {
            foreach (MorphMotion morphMotion in morphMotions)
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
            UVMorphData data = Morphs[morphName];
            foreach (UVMorphOffset uvMorphOffset in data.MorphOffsets)
            {
                switch (targetMorph)
                {
                    case MorphType.UV:
                        bufferManager.InputVerticies[uvMorphOffset.VertexIndex].UV =model.VertexList.Vertexes[uvMorphOffset.VertexIndex].UV+new Vector2(uvMorphOffset.UVOffset.X, uvMorphOffset.UVOffset.Y) * progress;
                        break;
                    case MorphType.UV_Additional1:
                        bufferManager.InputVerticies[uvMorphOffset.VertexIndex].AddUV1= model.VertexList.Vertexes[uvMorphOffset.VertexIndex].AdditionalUV[0] + uvMorphOffset.UVOffset * progress;
                        break;
                    case MorphType.UV_Additional2:
                        bufferManager.InputVerticies[uvMorphOffset.VertexIndex].AddUV2 = model.VertexList.Vertexes[uvMorphOffset.VertexIndex].AdditionalUV[1] + uvMorphOffset.UVOffset * progress;
                        break;
                    case MorphType.UV_Additional3:
                        bufferManager.InputVerticies[uvMorphOffset.VertexIndex].AddUV3 = model.VertexList.Vertexes[uvMorphOffset.VertexIndex].AdditionalUV[2] + uvMorphOffset.UVOffset * progress;
                        break;
                    case MorphType.UV_Additional4:
                        bufferManager.InputVerticies[uvMorphOffset.VertexIndex].AddUV4 = model.VertexList.Vertexes[uvMorphOffset.VertexIndex].AdditionalUV[3] + uvMorphOffset.UVOffset * progress;
                        break;
                    default:
                        throw new ArgumentOutOfRangeException("不適切なモーフタイプが渡されました");
                }

            }
            return true;
        }
    }
}
