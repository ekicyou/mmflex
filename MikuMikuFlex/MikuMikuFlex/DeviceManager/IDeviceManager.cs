using System;
using SlimDX.Direct3D11;
using SlimDX.DXGI;
using Device = SlimDX.Direct3D11.Device;

namespace MMF.DeviceManager
{
    public interface IDeviceManager : IDisposable
    {
        /// <summary>
        ///     初期化されたデバイス
        /// </summary>
        Device Device { get; }

        /// <summary>
        /// Direct2D/DirectWrite用DX10デバイス
        /// </summary>
        SlimDX.Direct3D10.Device Device10 { get; }

        FeatureLevel DeviceFeatureLevel { get; }


        /// <summary>
        ///     初期化されたデバイスコンテキスト
        /// </summary>
        DeviceContext Context { get; }

        /// <summary>
        /// 描画に利用しているアダプター
        /// </summary>
        Adapter CurrentAdapter { get; }

        Factory1 Factory { get; set; }

        /// <summary>
        /// ラスタライザステート
        /// </summary>
        //RasterizerStateDescription RasterizerStateDesc { get; set; }

        /// <summary>
        ///     コントロールに合わせて読み込む
        /// </summary>
        /// <param name="control"></param>
        void Load();
    }
}