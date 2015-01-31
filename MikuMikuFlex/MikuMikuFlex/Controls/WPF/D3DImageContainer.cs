using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Interop;
using SlimDX.Direct3D11;
using SlimDX.Direct3D9;
using Resource = SlimDX.DXGI.Resource;

namespace MMF.Controls.WPF
{
    /// <summary>
    ///     テクスチャの内容をWPFに表示できるようにするためのクラス
    /// </summary>
    public class D3DImageContainer : D3DImage, IDisposable
    {
        private static int ActiveImageCount;

        private static Direct3DEx D3DContext;

        private static DeviceEx D3DDevice;

        private Texture SharedTexture;

        private Surface surface;

        public D3DImageContainer()
        {
            InitD3D9();
            ActiveImageCount++;
        }

        public void Dispose()
        {
            SetBackBufferSlimDX(null);
            if (SharedTexture != null)
            {
                SharedTexture.Dispose();
                SharedTexture = null;
            }

            ActiveImageCount--;
            ShutdownD3D9();
        }

        [DllImport("user32.dll", SetLastError = false)]
        private static extern IntPtr GetDesktopWindow();

        public void SetBackBufferSlimDX(Texture2D texture)
        {
            //古い共有テクスチャは削除する
            if (SharedTexture != null)
            {
                SharedTexture.Dispose();
                SharedTexture = null;
            }

            if (IsShareable(texture))
            {
                Format format = TranslateFormat(texture);
                if (format == Format.Unknown) throw new InvalidDataException("対応していないテクスチャフォーマットが指定されました。");

                IntPtr handle = GetSharedHandle(texture);
                if (handle == IntPtr.Zero)
                    throw new InvalidDataException("共有ハンドルの生成に失敗");

                SharedTexture = new Texture(D3DDevice, texture.Description.Width, texture.Description.Height, 1,
                    Usage.RenderTarget, format, Pool.Default, ref handle);

                surface = SharedTexture.GetSurfaceLevel(0);
                {
                    Lock();
                    SetBackBuffer(D3DResourceType.IDirect3DSurface9, surface.ComPointer);
                    Unlock();
                }
            }
            else if (texture != null)
            {
                throw new InvalidDataException("テクスチャはResourceOptioFlags.Sharedをつけて作成されなければなりません。");
            }
        }

        private IntPtr GetSharedHandle(Texture2D texture)
        {
            var resource = new Resource(texture);
            IntPtr result = resource.SharedHandle;
            resource.Dispose();
            return result;
        }

        private Format TranslateFormat(Texture2D texture)
        {
            switch (texture.Description.Format)
            {
                case SlimDX.DXGI.Format.R10G10B10A2_UNorm:
                    return Format.A2B10G10R10;
                case SlimDX.DXGI.Format.R16G16B16A16_Float:
                    return Format.A16B16G16R16F;
                case SlimDX.DXGI.Format.B8G8R8A8_UNorm:
                case SlimDX.DXGI.Format.B8G8R8A8_UNorm_SRGB:
                    return Format.A8R8G8B8;
                default:
                    return Format.Unknown;
            }
        }

        /// <summary>
        ///     共有可能テクスチャかどうかチェックする
        /// </summary>
        /// <param name="texture"></param>
        /// <returns></returns>
        private bool IsShareable(Texture2D texture)
        {
            if (texture == null) return false;
            return (texture.Description.OptionFlags & ResourceOptionFlags.Shared) != 0;
        }

        /// <summary>
        ///     必要ありそうなら、DirectX9を初期化する。(WPF用)
        /// </summary>
        private void InitD3D9()
        {
            if (ActiveImageCount == 0)
            {
                D3DContext = new Direct3DEx();

                var presentParam = new PresentParameters();
                presentParam.Windowed = true;
                presentParam.SwapEffect = SwapEffect.Discard;
                presentParam.DeviceWindowHandle = GetDesktopWindow();
                presentParam.PresentationInterval = PresentInterval.Immediate;

                D3DDevice = new DeviceEx(D3DContext, 0, DeviceType.Hardware, IntPtr.Zero,
                    CreateFlags.HardwareVertexProcessing | CreateFlags.Multithreaded | CreateFlags.FpuPreserve,
                    presentParam);
                D3DDevice.SetRenderState(RenderState.AlphaBlendEnable, false);
            }
        }

        private void ShutdownD3D9()
        {
            if (ActiveImageCount == 0)
            {
                if (SharedTexture != null)
                {
                    SharedTexture.Dispose();
                    SharedTexture = null;
                }
                if (D3DDevice != null)
                {
                    D3DDevice.Dispose();
                    D3DDevice = null;
                }
                if (D3DContext != null)
                {
                    D3DContext.Dispose();
                    D3DContext = null;
                }
            }
        }

        public void InvalidateD3DImage()
        {
            if (SharedTexture != null)
            {
                Lock();
                AddDirtyRect(new Int32Rect(0, 0, PixelWidth, PixelHeight));
                Unlock();
            }
        }
    }
}