using System;
using SlimDX;
using SlimDX.Direct3D11;
using Buffer = SlimDX.Direct3D11.Buffer;

namespace MMF.MME.VariableSubscriber.ConstantSubscriber.ConstantBufferManager
{
    public abstract class ConstantBufferManagerBase<T> : IDisposable where T : struct
    {
        protected DataBox BufferDataBox;

        public Buffer ConstantBuffer;
        protected Device device;

        protected EffectConstantBuffer target;

        public void Dispose()
        {
            ConstantBuffer.Dispose();
        }

        public void Initialize(Device device, EffectConstantBuffer effectVariable, int size, T obj)
        {
            this.device = device;
            this.target = effectVariable;
            BufferDataBox = new DataBox(0, 0, new DataStream(new[] {obj}, true, true));
            ConstantBuffer = new Buffer(device, new BufferDescription
            {
                SizeInBytes = size,
                BindFlags = BindFlags.ConstantBuffer
            });
            OnInitialize();
        }

        protected abstract void OnInitialize();

        public abstract void ApplyToEffect(T obj);

        protected void WriteToBuffer(T obj)
        {
            BufferDataBox.Data.WriteRange(new[] {obj});
            BufferDataBox.Data.Position = 0;
            device.ImmediateContext.UpdateSubresource(BufferDataBox, ConstantBuffer, 0);
        }

        /// <summary>
        ///     指定した名前の定数バッファにバッファをセットします
        /// </summary>
        /// <param name="cbufferName"></param>
        /// <param name="buffer"></param>
        protected void SetConstantBuffer()
        {
            target.ConstantBuffer = ConstantBuffer;
        }
    }
}