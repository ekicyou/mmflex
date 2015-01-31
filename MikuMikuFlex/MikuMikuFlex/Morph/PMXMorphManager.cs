using System.Collections.Generic;
using MMDFileParser.PMXModelParser;
using MMF.Model;
using MMF.Model.PMX;
using MMF.Motion;

namespace MMF.Morph
{
    /// <summary>
    /// PMX標準のモーフを管理するクラス
    /// </summary>
    public class PMXMorphManager : IMorphManager
    {
        public PMXMorphManager(PMXModel model)
        {
            MMDMorphs.Add(new VertexMorphProvider(model.Model,model.BufferManager));
            MMDMorphs.Add(new BoneMorphProvider(model));
            MMDMorphs.Add(new MaterialMorphProvider(model));
            MMDMorphs.Add(new GroupMorphProvider(model,this));
            MMDMorphs.Add(new UVMorphProvider(model,MorphType.UV));
            MMDMorphs.Add(new UVMorphProvider(model,MorphType.UV_Additional1));
            MMDMorphs.Add(new UVMorphProvider(model,MorphType.UV_Additional2));
            MMDMorphs.Add(new UVMorphProvider(model, MorphType.UV_Additional3));
            MMDMorphs.Add(new UVMorphProvider(model, MorphType.UV_Additional4));
        }

        public List<IMorphProvider> MMDMorphs=new List<IMorphProvider>();

        private Dictionary<string,float> morphProgresses=new Dictionary<string, float>();

        public float getMorphProgress(string morphName)
        {
            return morphProgresses[morphName];
        }

        /// <summary>
        /// モーフのモーションから再生
        /// </summary>
        /// <param name="frame"></param>
        /// <param name="morphs"></param>
        public void ApplyMorphProgress(float frameNumber, IEnumerable<MorphMotion> morphMotions)
        {
            foreach (var morphMotion in morphMotions)
            {
                if (morphProgresses.ContainsKey(morphMotion.MorphName))
                {
                    morphProgresses[morphMotion.MorphName] = morphMotion.GetMorphValue(frameNumber);
                }
                else
                {
                    morphProgresses.Add(morphMotion.MorphName,morphMotion.GetMorphValue(frameNumber));
                }
            }
            foreach (IMorphProvider mmdMorphManager in MMDMorphs)
            {
                mmdMorphManager.ApplyMorphProgress(frameNumber, morphMotions);
            }
        }

        /// <summary>
        /// 指定したモーフのフレームをセットする
        /// </summary>
        /// <param name="frame"></param>
        /// <param name="morphName"></param>
        public void ApplyMorphProgress(float frame, string morphName)
        {
            if (morphProgresses.ContainsKey(morphName))
            {
                morphProgresses[morphName] = frame;
            }
            else
            {
                morphProgresses.Add(morphName, frame);
            }
            foreach (IMorphProvider mmdMorphManager in MMDMorphs)
            {
                if(mmdMorphManager.ApplyMorphProgress(frame,morphName))return;
            }
        }

        public void UpdateFrame()
        {
            foreach (var mmdMorphProvider in MMDMorphs)
            {
                mmdMorphProvider.UpdateFrame();
            }
        }
    }
}
