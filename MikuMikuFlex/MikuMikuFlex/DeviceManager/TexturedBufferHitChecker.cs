using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Assimp;
using BulletSharp.SoftBody;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;
using Debug = System.Diagnostics.Debug;
using MapFlags = SlimDX.Direct3D11.MapFlags;
using Resource = SlimDX.Direct3D11.Resource;

namespace MMF.DeviceManager
{
    /// <summary>
    /// 機種依存防止のため、テクスチャを無駄に２つ作ってるけどあまりよくない気がする。
    /// </summary>
    public class TexturedBufferHitChecker:IDisposable
    {
        private RenderContext renderContext;

        private TargetContextBase screenContext;

        private Texture2D targetUIntTexture;

        private Texture2D targetFloatTexture;

        private RenderTargetView floatRenderTarget;

        private RenderTargetView uintRenderTarget;

        private DepthStencilView depthTarget;

        public Point CheckPoint { get; set; }

        public bool IsMouseDown { get; set; }

        private Size currentSize;


        /// <summary>
        /// あたるかどうかチェックするリスト
        /// </summary>
        public List<IHitTestable> CheckTargets=new List<IHitTestable>(); 

        public TexturedBufferHitChecker(RenderContext context,TargetContextBase targetScreenContext)
        {
            renderContext = context;
            screenContext = targetScreenContext;
            targetUIntTexture = renderContext.CreateTexture2D(new Texture2DDescription()
            {
                ArraySize = 1,
                CpuAccessFlags = CpuAccessFlags.Read,
                Usage = ResourceUsage.Staging,
                Format = Format.R32_UInt,
                OptionFlags = ResourceOptionFlags.None,
                BindFlags = BindFlags.None,
                Height = 1,
                Width = 1,
                MipLevels = 1,
                SampleDescription = new SampleDescription(1, 0)
            });
            targetFloatTexture = renderContext.CreateTexture2D(new Texture2DDescription()
            {
                ArraySize = 1,
                CpuAccessFlags = CpuAccessFlags.Read,
                Usage = ResourceUsage.Staging,
                Format = Format.R32_Float,
                OptionFlags = ResourceOptionFlags.None,
                BindFlags = BindFlags.None,
                Height = 1,
                Width = 1,
                MipLevels = 1,
                SampleDescription = new SampleDescription(1, 0)
            });
            Resize(new Size(100,100));
        }

        public void Resize(Size size)
        {
            if(floatRenderTarget!=null&&!floatRenderTarget.Disposed)floatRenderTarget.Dispose();
            if (depthTarget != null && !depthTarget.Disposed) depthTarget.Dispose();
            if (floatRenderTarget != null && !floatRenderTarget.Disposed) floatRenderTarget.Dispose();
            if (uintRenderTarget != null && !uintRenderTarget.Disposed) uintRenderTarget.Dispose();
            using (Texture2D renderTexture=renderContext.CreateTexture2D(new Texture2DDescription()
            {
                CpuAccessFlags = CpuAccessFlags.None,Usage = ResourceUsage.Default,Format = Format.R32_Float,OptionFlags = ResourceOptionFlags.None,SampleDescription = new SampleDescription(1,0),Width = size.Width,Height = size.Height,BindFlags = BindFlags.RenderTarget,MipLevels = 1,ArraySize = 1
            }))
            {
                this.floatRenderTarget=new RenderTargetView(renderContext.DeviceManager.Device,renderTexture);
            }
            using (Texture2D renderTexture = renderContext.CreateTexture2D(new Texture2DDescription()
            {
                CpuAccessFlags = CpuAccessFlags.None,
                Usage = ResourceUsage.Default,
                Format = Format.R32_UInt,
                OptionFlags = ResourceOptionFlags.None,
                SampleDescription = new SampleDescription(1, 0),
                Width = size.Width,
                Height = size.Height,
                BindFlags = BindFlags.RenderTarget,
                MipLevels = 1,
                ArraySize = 1
            }))
            {
                this.uintRenderTarget = new RenderTargetView(renderContext.DeviceManager.Device, renderTexture);
            }
            using (Texture2D depthtexture=renderContext.CreateTexture2D(new Texture2DDescription()
            {
                CpuAccessFlags = CpuAccessFlags.None,Usage = ResourceUsage.Default,Format = Format.D32_Float,OptionFlags = ResourceOptionFlags.None,SampleDescription = new SampleDescription(1,0),Width = size.Width,BindFlags = BindFlags.DepthStencil,Height = size.Height,MipLevels = 1,ArraySize = 1
            }))
            {
                this.depthTarget=new DepthStencilView(renderContext.DeviceManager.Device,depthtexture);
            }
            CheckPoint=new Point();
            currentSize = size;
        }

        public void CheckTarget()
        {
            if(currentSize.Width<CheckPoint.X+2||currentSize.Height<CheckPoint.Y+2||CheckPoint.X<0||CheckPoint.Y<0)return;
            //チェック用ターゲットの描画(UINT)
            renderContext.DeviceManager.Device.ImmediateContext.OutputMerger.SetTargets(depthTarget,floatRenderTarget);
            renderContext.DeviceManager.Device.ImmediateContext.ClearDepthStencilView(depthTarget,DepthStencilClearFlags.Depth, 1,0);
            renderContext.DeviceManager.Device.ImmediateContext.ClearRenderTargetView(floatRenderTarget,
                new Color4(0, 0, 0, 0));
            for (int index = 0; index < CheckTargets.Count; index++)
            {
                var checkTarget = CheckTargets[index];
                if(checkTarget.Visibility)checkTarget.RenderHitTestBuffer(index+1);
            }
            renderContext.DeviceManager.Device.ImmediateContext.Flush();
            //チェック用ターゲットの描画(FLOAT)
            renderContext.DeviceManager.Device.ImmediateContext.OutputMerger.SetTargets(depthTarget, uintRenderTarget);
            renderContext.DeviceManager.Device.ImmediateContext.ClearDepthStencilView(depthTarget, DepthStencilClearFlags.Depth, 1, 0);
            renderContext.DeviceManager.Device.ImmediateContext.ClearRenderTargetView(uintRenderTarget,
                new Color4(0, 0, 0, 0));
            for (int index = 0; index < CheckTargets.Count; index++)
            {
                var checkTarget = CheckTargets[index];
                if(checkTarget.Visibility)checkTarget.RenderHitTestBuffer(index + 1);
            }
            renderContext.DeviceManager.Device.ImmediateContext.Flush();
            //GPU⇒CPUテクスチャ転送
            renderContext.DeviceManager.Context.CopySubresourceRegion(uintRenderTarget.Resource, 0,
                new ResourceRegion(CheckPoint.X, CheckPoint.Y, 0, CheckPoint.X + 1, CheckPoint.Y + 1, 1), targetUIntTexture,
                0, 0, 0, 0);
            renderContext.DeviceManager.Context.CopySubresourceRegion(floatRenderTarget.Resource, 0,
                new ResourceRegion(CheckPoint.X, CheckPoint.Y, 0, CheckPoint.X + 1, CheckPoint.Y + 1, 1), targetFloatTexture,
                0, 0, 0, 0);
            DataBox floatMapResource = renderContext.DeviceManager.Context.MapSubresource(targetFloatTexture, 0, MapMode.Read,
                MapFlags.None);
            DataBox uintMapResource = renderContext.DeviceManager.Context.MapSubresource(targetUIntTexture, 0, MapMode.Read,
    MapFlags.None);
            float ud = uintMapResource.Data.Read<float>();//なぜかここがfloatだとうまくいく
            float fd = floatMapResource.Data.Read<float>();
            for (int index = 0; index < CheckTargets.Count; index++)
            {
                var checkTarget = CheckTargets[index];
                checkTarget.HitTestResult((ud ==(index+1)||fd==(index+1))&&checkTarget.Visibility,IsMouseDown,CheckPoint);
            }

            // Debug.WriteLine(CheckPoint.ToString()+b);
            //for (int index = 0; index < CheckTargets.Count; index++)
           // {
           //     var checkTarget = CheckTargets[index];
           //     checkTarget.HitTestResult((index + 1) == b, IsMouseDown, CheckPoint);
            //}
            renderContext.DeviceManager.Context.UnmapSubresource(targetUIntTexture,0);
            renderContext.DeviceManager.Context.UnmapSubresource(targetFloatTexture, 0);
        }


        public void Dispose()
        {
            if(targetUIntTexture!=null&&!targetUIntTexture.Disposed)targetUIntTexture.Dispose();
            if (targetFloatTexture != null && !targetFloatTexture.Disposed) targetFloatTexture.Dispose();
            if(depthTarget!=null&&!depthTarget.Disposed)depthTarget.Dispose();
            if(floatRenderTarget!=null&&!floatRenderTarget.Disposed)floatRenderTarget.Dispose();
            if (uintRenderTarget != null && !uintRenderTarget.Disposed) uintRenderTarget.Dispose();
        }
    }
}
