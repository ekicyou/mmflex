using System.IO;
using MMDFileParser.PMXModelParser;
using MMF.Bone;

namespace MMF.Model.PMX
{
    public class PMXModelWithPhysics : PMXModel
    {
        public PMXModelWithPhysics(ModelData modeldata, ISubresourceLoader subResourceLoader, string filename)
            : base(modeldata, subResourceLoader, filename)
        {
        }

        public new static PMXModel OpenLoad(string filePath, RenderContext context)
        {
            PMXModelWithPhysics model = (PMXModelWithPhysics) FromFile(filePath);
            model.Load(context);
            return model;
        }

        public new static PMXModel FromFile(string filePath)
        {
            string folder = Path.GetDirectoryName(filePath);
            return FromFile(filePath, folder);
        }

        public new static PMXModel FromFile(string filePath, string textureFolder)
        {
            return FromFile(filePath, new BasicSubresourceLoader(textureFolder));
        }

        public new static PMXModel FromFile(string filePath, ISubresourceLoader loader)
        {
            using (FileStream fs = File.OpenRead(filePath))
            {
                return new PMXModelWithPhysics(ModelData.GetModel(fs), loader, Path.GetFileName(filePath));
            }
        }

        public override void Dispose()
        {
            Skinning.Dispose();
            base.Dispose();
        }

        protected override ISkinningProvider InitializeSkinning()
        {
            PMXSkeletonWithPhysics skel = new PMXSkeletonWithPhysics(Model);
            return skel;
        }
    }
}