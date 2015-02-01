using System.Collections.Generic;
using Assimp;
using MMF.MME.VariableSubscriber.MaterialSubscriber;
using MMF.Utility;
using SlimDX;
using SlimDX.Direct3D11;

namespace MMF.Model.Assimp
{
    public class AssimpSubset:ISubset
    {
        private Mesh mesh;

        private Buffer vBuffer;
        private RenderContext context;

        public AssimpSubset(RenderContext context,ISubresourceLoader loader,IDrawable drawable,Scene scene,int index)
        {
            this.SubsetId = index;
            this.context = context;
            this.DoCulling =false;
            Drawable = drawable;
             Material material = scene.Materials[scene.Meshes[index].MaterialIndex];
            mesh = scene.Meshes[index];
            Initialize();
            this.MaterialInfo = MaterialInfo.FromMaterialData(drawable,material,context,loader);
        }

        private void Initialize()
        {
            
            List<BasicInputLayout> verticies=new List<BasicInputLayout>();
            foreach (var face in mesh.Faces)
            {
                foreach (int vIndex in face.Indices)
                {
                    BasicInputLayout input = new BasicInputLayout();
                    input.Position = mesh.Vertices[vIndex].ToSlimDXVec4().InvX();
                    input.Normal = mesh.Normals[vIndex].ToSlimDX();
                    if (mesh.TextureCoordinateChannelCount > 0 && mesh.TextureCoordinateChannels[0] != null)
                    {
                        Vector3 vec = mesh.TextureCoordinateChannels[0][vIndex].ToSlimDX();
                        input.UV = new Vector2(vec.X, 1-vec.Y);
                    }
                    input.BoneWeight1 = 1f;
                    input.BoneIndex1 = 0;
                    verticies.Add(input);
                }
            }
            vBuffer = CGHelper.CreateBuffer(verticies, context.DeviceManager.Device, BindFlags.VertexBuffer);
        }

        public MaterialInfo MaterialInfo { get; private set; }
        public int SubsetId { get; private set; }
        public IDrawable Drawable { get; set; }
        public bool DoCulling { get; private set; }

        public void Draw(Device device)
        {
            device.ImmediateContext.InputAssembler.SetVertexBuffers(0,
                new VertexBufferBinding(vBuffer, BasicInputLayout.SizeInBytes, 0));
            device.ImmediateContext.Draw(mesh.FaceCount*3,0);
        }

        public void Dispose()
        {
            if(vBuffer!=null&&!vBuffer.Disposed)vBuffer.Dispose();
            MaterialInfo.Dispose();
        }
    }
}
