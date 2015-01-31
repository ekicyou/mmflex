using System.IO;

namespace MMDFileParser.PMXModelParser
{
    public class ModelData
    {
        public Header Header { get; private set; }

        public ModelInfo ModelInfo { get; private set; }

        public VertexList VertexList { get; private set; }

        public SurfaceList SurfaceList { get; private set; }

        public TextureList TextureList { get; private set; }

        public MaterialList MaterialList { get; private set; }

        public BoneList BoneList { get; private set; }

        public MorphList MorphList { get; private set; }

        public ShowFrameList ShowFrameList { get; private set; }

        public RigidBodyList RigidBodyList { get; private set; }

        public JointList JointList { get; private set; }

        public static ModelData GetModel(FileStream fs)
        {
            ModelData model = new ModelData();
            model.Header = Header.getHeader(fs);
            model.ModelInfo = ModelInfo.getModelInfo(fs, model.Header);
            model.VertexList = VertexList.getVertexList(fs, model.Header);
            model.SurfaceList = SurfaceList.getSurfaceList(fs, model.Header);
            model.TextureList = TextureList.getTextureList(fs, model.Header);
            model.MaterialList = MaterialList.getMaterialList(fs, model.Header);
            model.BoneList = BoneList.getBoneList(fs, model.Header);
            model.MorphList = MorphList.getMorphList(fs, model.Header);
            model.ShowFrameList = ShowFrameList.getShowFrameList(fs, model.Header);
            model.RigidBodyList = RigidBodyList.GetRigidBodyList(fs, model.Header);
            model.JointList = JointList.getJointList(fs, model.Header);
            return model;
        }
    }
}