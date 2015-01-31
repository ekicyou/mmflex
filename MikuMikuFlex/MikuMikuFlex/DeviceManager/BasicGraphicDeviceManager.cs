#region

using System;
using System.Diagnostics;
using System.Windows.Forms;
using MMF.MME;
using SlimDX;
using SlimDX.Direct3D11;
using SlimDX.DXGI;
using Debug = System.Diagnostics.Debug;
using Device = SlimDX.Direct3D11.Device;
using Device1 = SlimDX.Direct3D10_1.Device1;
using DriverType = SlimDX.Direct3D10.DriverType;

#endregion

namespace MMF.DeviceManager
{
    /// <summary>
    ///     通常のデバイスの初期化などを行うクラス。
    ///     変更したい場合は継承するといい
    /// </summary>
    public class BasicGraphicDeviceManager : IDeviceManager
    {
        // private RasterizerStateDescription rasterizerStateDesc;

        public FeatureLevel DeviceFeatureLevel { get; private set; }

        /// <summary>
        ///     デバイス
        /// </summary>
        public Device Device { get; private set; }

        /// <summary>
        ///     Device.ImmideateContext
        /// </summary>
        public DeviceContext Context { get; private set; }

        /// <summary>
        ///     Direct2D/DirectWrite用DX10デバイス
        /// </summary>
        public SlimDX.Direct3D10.Device Device10 { get; private set; }

        /// <summary>
        ///     描画に利用しているアダプター
        /// </summary>
        public Adapter CurrentAdapter { get; private set; }

        public void Load()
        {
            Load(false,DeviceCreationFlags.None,SlimDX.Direct3D10.DeviceCreationFlags.BgraSupport);
        }

        /// <summary>
        ///     デバイスの初期化処理
        /// </summary>
        /// <param name="control">適用するコントロールへの参照</param>
        public void Load(bool needDX11=false,DeviceCreationFlags dx11flag=DeviceCreationFlags.None,SlimDX.Direct3D10.DeviceCreationFlags dx10flag_for2DDraw=SlimDX.Direct3D10.DeviceCreationFlags.BgraSupport)
        {
            ApplyDebugFlags(ref dx11flag,ref dx10flag_for2DDraw);
            Factory = new Factory1();
            CurrentAdapter = Factory.GetAdapter1(0);
            //スワップチェーンの初期化
            try
            {
                Device = new Device(CurrentAdapter, dx11flag,
                    new[] {FeatureLevel.Level_11_0});
            }
            catch (Direct3D11Exception)
            {
                if (needDX11)
                    throw new NotSupportedException("DX11がサポートされていません。DX10.1で初期化するにはLoadの第一引数needDraw=falseとして下さい。");
                try
                {
                    Device = new Device(CurrentAdapter, dx11flag, new[] {FeatureLevel.Level_10_0});
                }
                catch (Direct3D11Exception)
                {
                    throw new NotSupportedException("DX11,DX10.1での初期化を試みましたが、両方ともサポートされていません。");
                }
            }

            DeviceFeatureLevel = Device.FeatureLevel;
            Context = Device.ImmediateContext;
            SampleDescription sampleDesc = new SampleDescription(1, 0);
#if VSG_DEBUG
#else
            Device10 = new Device1(CurrentAdapter, DriverType.Hardware,
                dx10flag_for2DDraw, SlimDX.Direct3D10_1.FeatureLevel.Level_9_3);
#endif
            MMEEffectManager.IniatializeMMEEffectManager(this);
        }

        [Conditional("DEBUG")]
        private void ApplyDebugFlags(ref DeviceCreationFlags dx11flag, ref SlimDX.Direct3D10.DeviceCreationFlags dx10flag_for2DDraw)
        {
            Debug.Print("デバイスはデバッグモードで作成されました。");
            //dx11flag = dx11flag | DeviceCreationFlags.Debug;
            dx10flag_for2DDraw = dx10flag_for2DDraw | SlimDX.Direct3D10.DeviceCreationFlags.BgraSupport;
        }

        public virtual void Dispose()
        {
            if (!Context.Disposed && Context.Rasterizer.State != null && !Context.Rasterizer.State.Disposed)
                Context.Rasterizer.State.Dispose();
            if (Device != null && !Device.Disposed) Device.Dispose();
            if (Device10 != null && !Device10.Disposed) Device10.Dispose();
            if (CurrentAdapter != null && !CurrentAdapter.Disposed) CurrentAdapter.Dispose();
            if (Factory != null && !Factory.Disposed) Factory.Dispose();
        }

        public Factory1 Factory { get; set; }

        protected virtual void PostLoad(Control control)
        {
        }

        /// <summary>
        ///     スワップチェーンの設定を取得します。
        ///     スワップチェーンの設定を変えたい場合は、オーバーライドしてください。
        /// </summary>
        /// <param name="control">適応するコントロールへの参照</param>
        /// <returns>スワップチェーンの設定</returns>
        protected virtual SwapChainDescription getSwapChainDescription(Control control, SampleDescription sampDesc)
        {
            return new SwapChainDescription
            {
                BufferCount = 2,
                Flags = SwapChainFlags.AllowModeSwitch,
                IsWindowed = true,
                ModeDescription = new ModeDescription
                {
                    Format = Format.R8G8B8A8_UNorm,
                    Height = control.Height,
                    Width = control.Width,
                    RefreshRate = new Rational(60, 1)
                },
                OutputHandle = control.Handle,
                SampleDescription = sampDesc,
                SwapEffect = SwapEffect.Discard,
                Usage = Usage.RenderTargetOutput
            };
        }

        /// <summary>
        ///     深度ステンシルバッファの設定を取得します。
        ///     深度ステンシルバッファの設定を変えたい場合はオーバーライドしてください。
        /// </summary>
        /// <param name="control">適用するコントロールへの参照</param>
        /// <returns>深度ステンシルバッファ用のTexture2Dの設定</returns>
        protected virtual Texture2DDescription getDepthBufferTexture2DDescription(Control control,
            SampleDescription desc)
        {
            return new Texture2DDescription
            {
                ArraySize = 1,
                BindFlags = BindFlags.DepthStencil,
                Format = Format.D32_Float,
                Width = control.Width,
                Height = control.Height,
                MipLevels = 1,
                SampleDescription = desc
            };
        }

        /// <summary>
        ///     ビューポートの内容を取得します。
        /// </summary>
        /// <param name="control">適用するコントロールへの参照</param>
        /// <returns>設定するビューポート</returns>
        protected virtual Viewport getViewport(Control control)
        {
            return new Viewport
            {
                Width = control.Width,
                Height = control.Height,
                MaxZ = 1
            };
        }

        /// <summary>
        ///     ラスタライザの設定を取得します。
        /// </summary>
        /// <param name="control">適用するコントロールへの参照</param>
        /// <returns>設定するビューポート</returns>
        protected virtual RasterizerStateDescription getRasterizerStateDescription(Control control)
        {
            return new RasterizerStateDescription
            {
                CullMode = CullMode.Back,
                FillMode = FillMode.Solid,
                DepthBias = 0,
                DepthBiasClamp = 0,
                IsAntialiasedLineEnabled = true,
                IsDepthClipEnabled = true,
                IsFrontCounterclockwise = false,
                IsMultisampleEnabled = true,
                IsScissorEnabled = false,
                SlopeScaledDepthBias = 0
            };
        }
    }
}