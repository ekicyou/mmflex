using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMF.Matricies;
using MMF.Matricies.Camera;
using SlimDX.Direct3D11;

namespace MMF.DeviceManager
{
    public abstract class TargetContextBase:ITargetContext
    {
        public TargetContextBase(RenderContext renderContext)
        {
            this.Context = renderContext;
        }

        /// <summary>
        /// リサイズ時などのためにDepthtargetViewとRenderTargetViewをDisposeします。
        /// nullの場合はスキップされます。
        /// </summary>
        protected void DisposeTargetViews()
        {
            if (RenderTargetView != null && !RenderTargetView.Disposed) RenderTargetView.Dispose();
            if (DepthTargetView != null && !DepthTargetView.Disposed) DepthTargetView.Dispose();
        }

        /// <summary>
        /// 指定したSTAGINGテクスチャに対してRenderTargetをコピーする
        /// </summary>
        /// <param name="stagingTexture"></param>
        public void CopyRenderTarget(Texture2D stagingTexture)
        {
            Context.DeviceManager.Context.CopyResource(RenderTargetView.Resource,stagingTexture);
        }

        /// <summary>
        /// 指定したSTAGINGテクスチャに対してRenderTargetをコピーする
        /// </summary>
        /// <param name="stagingTexture">コピー先</param>
        /// <param name="left">左座標</param>
        /// <param name="top">上座標</param>
        /// <param name="height">高さ</param>
        /// <param name="width">幅</param>
        public void CopyRegionOfRenderTarget(Texture2D stagingTexture,int left,int top,int height,int width)
        {
            Context.DeviceManager.Context.CopySubresourceRegion(RenderTargetView.Resource, 0,
                new ResourceRegion(left, top, 0, left + width, top + height, 1), stagingTexture, 0, 0, 0, 0);
        }

        public abstract void Dispose();
        public RenderTargetView RenderTargetView { get; protected set; }
        public DepthStencilView DepthTargetView { get; protected set; }
        public abstract MatrixManager MatrixManager { get; set; }
        public abstract ICameraMotionProvider CameraMotionProvider { get; set; }
        public abstract WorldSpace WorldSpace { get; set; }
        public bool IsSelfShadowMode1 { get; protected set; }
        public bool IsEnabledTransparent { get; protected set; }
        public RenderContext Context { get; set; }
        public abstract void SetViewport();
    }
}
