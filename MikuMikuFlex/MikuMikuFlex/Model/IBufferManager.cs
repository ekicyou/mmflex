using System;
using SlimDX.Direct3D11;
using Buffer = SlimDX.Direct3D11.Buffer;

namespace MMF.Model
{
    public interface IBufferManager : IDisposable
    {
        Buffer VertexBuffer { get; }

        Buffer IndexBuffer { get; }

        BasicInputLayout[] InputVerticies
        {get; }

        bool NeedReset { get; set; }

        void RecreateVerticies();

        InputLayout VertexLayout { get; }

        void Initialize(object model, Device device, Effect effect);

        int VerticiesCount { get; }
    }
}