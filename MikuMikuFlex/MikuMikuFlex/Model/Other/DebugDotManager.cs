using System;
using System.Collections.Generic;
using MMF.Utility;
using SlimDX;
using SlimDX.Direct3D11;
using Buffer = SlimDX.Direct3D11.Buffer;

namespace MMF.Model.Other
{
    public class DebugDotManager
    {
        public Buffer VertexBuffer { get; private set; }

        public InputLayout VertexLayout { get; private set; }

        public Effect Effect { get; private set; }

        public const float dotlength = 0.8f;

        public DebugDotManager(RenderContext context)
        {
            this.Context = context;
            List<byte> listBuffer = new List<byte>();
            CGHelper.AddListBuffer(new Vector3(-dotlength/2, dotlength/2, 0), listBuffer);
            CGHelper.AddListBuffer(new Vector3(dotlength/2, dotlength/2, 0), listBuffer);
            CGHelper.AddListBuffer(new Vector3(-dotlength/2, -dotlength/2, 0), listBuffer);
            CGHelper.AddListBuffer(new Vector3(dotlength/2, dotlength/2, 0), listBuffer);
            CGHelper.AddListBuffer(new Vector3(dotlength/2, -dotlength/2, 0), listBuffer);
            CGHelper.AddListBuffer(new Vector3(-dotlength/2, -dotlength/2, 0), listBuffer);
            VertexBuffer = CGHelper.CreateBuffer(listBuffer, context.DeviceManager.Device, BindFlags.VertexBuffer);
            Effect = CGHelper.CreateEffectFx5("Shader\\debugDot.fx", context.DeviceManager.Device);
            RenderPass = Effect.GetTechniqueByIndex(0).GetPassByIndex(0);
            VertexLayout = new InputLayout(context.DeviceManager.Device,
                Effect.GetTechniqueByIndex(0).GetPassByIndex(0).Description.Signature, DebugDotInputLayout.InputElements);
        }

        public EffectPass RenderPass { get; set; }

        public RenderContext Context { get; set; }

        public void Draw(List<Vector3> positions,Vector4 color)
        {
            if(positions==null)return;
            Effect.GetVariableBySemantic("COLOR").AsVector().Set(color);
            for (int i = 0; i < positions.Count; i++)
            {
                Vector3 position = positions[i];
                Vector3 p2lp = Vector3.Normalize(this.Context.MatrixManager.ViewMatrixManager.CameraPosition - position);
                Vector3 axis = Vector3.Cross(new Vector3(0, 0, -1), p2lp);
                float angle = (float)Math.Acos(Vector3.Dot(new Vector3(0, 0, -1), p2lp));
                Quaternion quat = Quaternion.RotationAxis(axis, angle);
                DeviceContext Context = this.Context.DeviceManager.Context;
                Effect.GetVariableBySemantic("WORLDVIEWPROJECTION")
                    .AsMatrix()
                    .SetMatrix(this.Context.MatrixManager.makeWorldViewProjectionMatrix(new Vector3(1f), quat, position));
                Context.InputAssembler.SetVertexBuffers(0, new VertexBufferBinding(VertexBuffer, DebugDotInputLayout.SizeInBytes, 0));
                Context.InputAssembler.InputLayout = VertexLayout;
                Context.InputAssembler.PrimitiveTopology = PrimitiveTopology.TriangleList;
                RenderPass.Apply(Context);
                Context.Draw(6, 0);
            }
        }
}
}
