using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using MMF.DeviceManager;
using MMF.Sprite.D2D;
using MMF.Utility;
using SlimDX;
using SlimDX.Direct2D;
using SlimDX.Direct3D11;
using SlimDX.DirectWrite;
using SlimDX.DXGI;
using FeatureLevel = SlimDX.Direct2D.FeatureLevel;
using FontStyle = SlimDX.DirectWrite.FontStyle;
using Resource = SlimDX.DXGI.Resource;

namespace MMF.Sprite
{
    /// <summary>
    /// Direct2Dを利用するスプライト表示クラス
    /// 大量に書き込むときはGDISpriteBatchを超えるが、FPS表示程度の更新速度の場合コチラのほうが劣る。
    /// また、VisualStudioのグラフィックデバッガが使えなくなるため、その場合はGDISpriteBatchを利用するべき。
    /// </summary>
    public class D2DSpriteBatch:IDisposable
    {
        /// <summary>
        /// デバイスマネージャ
        /// </summary>
        IDeviceManager DeviceManager { get; set; }

        /// <summary>
        /// スプライト表示に利用するDX11側テクスチャ
        /// </summary>
        private Texture2D TextureD3D11 { get; set; }

        /// <summary>
        /// スプライト表示に利用するDX10側テクスチャ
        /// </summary>
        private SlimDX.Direct3D10.Texture2D TextureD3D10 { get;  set; }

        /// <summary>
        /// DX10とDX11のテクスチャの共有に利用するKeyedMutex
        /// </summary>
        private KeyedMutex MutexD3D10 { get; set; }

        /// <summary>
        /// DX10とDX11のテクスチャの共有に利用するKeyedMutex
        /// </summary>
        private KeyedMutex MutexD3D11 { get; set; }



        /// <summary>
        /// 描画対象
        /// </summary>
        public RenderTarget DWRenderTarget { get; private set; }


        /// <summary>
        /// スプライトのブレンドモード
        /// </summary>
        private BlendState TransParentBlendState { get; set; }

        /// <summary>
        /// ブレンドに利用するBlendState
        /// </summary>
        private BlendState state;

        /// <summary>
        /// スプライトの描画に利用するビュー行列
        /// </summary>
        private Matrix ViewMatrix;

        /// <summary>
        /// スプライトの描画に利用する射影行列
        /// </summary>
        private Matrix SpriteProjectionMatrix;

        /// <summary>
        /// スプライトの描画に利用するビューポート
        /// </summary>
        private Viewport spriteViewport;

        internal RenderContext context;

        public Rectangle FillRectangle
        {
            get
            {
                return new Rectangle(0,0,(int)TextureSize.X,(int)TextureSize.Y);
            }
        }

        /// <summary>
        /// スプライトのテクスチャの描画先頂点バッファ
        /// </summary>
        private SlimDX.Direct3D11.Buffer VertexBuffer { get; set; }

        /// <summary>
        /// スプライトの入力レイアウト//Oculus Rift開発キット
        /// </summary>
        private InputLayout VertexInputLayout { get; set; }

        /// <summary>
        /// スプライトの描画に利用するエフェクト
        /// </summary>
        private Effect SpriteEffect { get; set; }

        /// <summary>
        /// スプライトの描画に利用するパス
        /// </summary>
        private EffectPass renderPass { get; set; }


        /// <summary>
        /// スプライトのテクスチャ用のサンプラ
        /// </summary>
        private SamplerState sampler { get; set; }

        /// <summary>
        /// DirectWriteレンダーターゲットが再作成されたことを通知します
        /// </summary>
        public event EventHandler<EventArgs> RenderTargetRecreated;

        public event EventHandler<EventArgs> BatchDisposing;

        public D2DSpriteBatch(RenderContext context)
        {
            this.context = context;
            DeviceManager = context.DeviceManager;
            SpriteEffect = CGHelper.CreateEffectFx5FromResource("MMF.Resource.Shader.SpriteShader.fx", DeviceManager.Device);
            renderPass = SpriteEffect.GetTechniqueByIndex(0).GetPassByIndex(1);
            VertexInputLayout = new InputLayout(DeviceManager.Device, SpriteEffect.GetTechniqueByIndex(0).GetPassByIndex(0).Description.Signature, SpriteVertexLayout.InputElements);
            SamplerDescription desc = new SamplerDescription();
            desc.Filter = Filter.MinMagMipLinear;
            desc.AddressU = TextureAddressMode.Wrap;
            desc.AddressV = TextureAddressMode.Wrap;
            desc.AddressW = TextureAddressMode.Wrap;
            sampler = SamplerState.FromDescription(DeviceManager.Device, desc);
            BlendStateDescription blendDesc = new BlendStateDescription();
            blendDesc.AlphaToCoverageEnable = false;
            blendDesc.IndependentBlendEnable = false;
            for (int i = 0; i < blendDesc.RenderTargets.Length; i++)
            {
                blendDesc.RenderTargets[i].BlendEnable = true;
                blendDesc.RenderTargets[i].SourceBlend = BlendOption.SourceAlpha;
                blendDesc.RenderTargets[i].DestinationBlend = BlendOption.InverseSourceAlpha;
                blendDesc.RenderTargets[i].BlendOperation = BlendOperation.Add;
                blendDesc.RenderTargets[i].SourceBlendAlpha = BlendOption.One;
                blendDesc.RenderTargets[i].DestinationBlendAlpha = BlendOption.Zero;
                blendDesc.RenderTargets[i].BlendOperationAlpha = BlendOperation.Add;
                blendDesc.RenderTargets[i].RenderTargetWriteMask = ColorWriteMaskFlags.All;
            }
            state = BlendState.FromDescription(DeviceManager.Device, blendDesc);
            
            BlendStateDescription bsd=new BlendStateDescription();
            bsd.RenderTargets[0].BlendEnable = true;
            bsd.RenderTargets[0].SourceBlend = BlendOption.SourceAlpha;
            bsd.RenderTargets[0].DestinationBlend = BlendOption.InverseSourceAlpha;
            bsd.RenderTargets[0].BlendOperation = BlendOperation.Add;
            bsd.RenderTargets[0].SourceBlendAlpha = BlendOption.One;
            bsd.RenderTargets[0].DestinationBlendAlpha = BlendOption.Zero;
            bsd.RenderTargets[0].BlendOperationAlpha = BlendOperation.Add;
            bsd.RenderTargets[0].RenderTargetWriteMask = ColorWriteMaskFlags.All;
            TransParentBlendState = BlendState.FromDescription(DeviceManager.Device, bsd);
            Resize();
        }

        public void Resize()
        {
            Viewport vp = DeviceManager.Context.Rasterizer.GetViewports()[0];
            int width = (int) vp.Width;
            int height = (int) vp.Height;
            if(height==0||width==0)return;
            TextureSize = new Vector2(width, height);
            float w = width / 2f, h = height / 2f;
            List<byte> vertexBytes = new List<byte>();
            CGHelper.AddListBuffer(new Vector3(-w, h, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector2(0, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector3(w, h, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector2(1, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector3(-w, -h, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector2(0, 1), vertexBytes);
            CGHelper.AddListBuffer(new Vector3(w, h, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector2(1, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector3(w, -h, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector2(1, 1), vertexBytes);
            CGHelper.AddListBuffer(new Vector3(-w, -h, 0), vertexBytes);
            CGHelper.AddListBuffer(new Vector2(0, 1), vertexBytes);
            using (DataStream ds = new DataStream(vertexBytes.ToArray(), true, true))
            {
                BufferDescription bufDesc = new BufferDescription()
                {
                    BindFlags = BindFlags.VertexBuffer,
                    SizeInBytes = (int)ds.Length
                };
                if (VertexBuffer != null && !VertexBuffer.Disposed) VertexBuffer.Dispose();
                VertexBuffer = new SlimDX.Direct3D11.Buffer(DeviceManager.Device, ds, bufDesc);
            }
            SpriteProjectionMatrix = Matrix.OrthoLH(width, height, 0, 100);
            spriteViewport = new Viewport()
            {
                Width = width,
                Height = height,
                MaxZ = 1
            };

            
            ViewMatrix = Matrix.LookAtLH(new Vector3(0, 0, -1), new Vector3(0, 0, 0), new Vector3(0, 1, 0));
            
            //DirectX11用テクスチャの作成
            if (TextureD3D11 != null && !TextureD3D11.Disposed) TextureD3D11.Dispose();
            TextureD3D11 = new Texture2D(DeviceManager.Device, new Texture2DDescription()
            {
                Width = width,
                Height = height,
                MipLevels = 1,
                ArraySize = 1,
                Format = Format.B8G8R8A8_UNorm,
                SampleDescription = new SampleDescription(1, 0),
                Usage = ResourceUsage.Default,
                BindFlags = BindFlags.RenderTarget | BindFlags.ShaderResource,
                CpuAccessFlags = CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.KeyedMutex
            });
            //DX11のテクスチャリソースをDX10とシェアする
            Resource sharedResource = new Resource(TextureD3D11);
            if (TextureD3D10 != null && !TextureD3D10.Disposed) TextureD3D10.Dispose();
            TextureD3D10 = DeviceManager.Device10.OpenSharedResource<SlimDX.Direct3D10.Texture2D>(sharedResource.SharedHandle);
            if (MutexD3D10 != null && !MutexD3D10.Disposed) MutexD3D10.Dispose();
            if (MutexD3D11 != null && !MutexD3D11.Disposed) MutexD3D11.Dispose();
            MutexD3D10 = new KeyedMutex(TextureD3D10);
            MutexD3D11 = new KeyedMutex(TextureD3D11);
            sharedResource.Dispose();
            Surface surface = TextureD3D10.AsSurface();
            RenderTargetProperties rtp = new RenderTargetProperties();
            rtp.MinimumFeatureLevel = FeatureLevel.Direct3D10;
            rtp.Type = RenderTargetType.Hardware;
            rtp.Usage = RenderTargetUsage.None;
            rtp.PixelFormat = new PixelFormat(Format.Unknown, AlphaMode.Premultiplied);
            if (DWRenderTarget != null && !DWRenderTarget.Disposed) DWRenderTarget.Dispose();
            DWRenderTarget = RenderTarget.FromDXGI(context.D2DFactory, surface, rtp);
            surface.Dispose();
           
            if(RenderTargetRecreated!=null)RenderTargetRecreated(this,new EventArgs());
        }

        public void Begin()
        {
            if(TextureSize==Vector2.Zero)return;
            MutexD3D10.Acquire(0, 100);
            DWRenderTarget.BeginDraw();
            DWRenderTarget.Clear(new Color4(0,0,0,0));     
        }

        public void End()
        {
            if (TextureSize == Vector2.Zero) return;
            DWRenderTarget.EndDraw();
            MutexD3D10.Release(0);
            MutexD3D11.Acquire(0, 100);
            ShaderResourceView srv=new ShaderResourceView(DeviceManager.Device,TextureD3D11);
            BlendState lastBlendState =DeviceManager.Context.OutputMerger.BlendState;
            Viewport[] lastViewports = DeviceManager.Context.Rasterizer.GetViewports();
            //WVP行列をエフェクトに渡す
            SpriteEffect.GetVariableBySemantic("WORLDVIEWPROJECTION")
                .AsMatrix()
                .SetMatrix(ViewMatrix * SpriteProjectionMatrix);
            SpriteEffect.GetVariableBySemantic("SPRITETEXTURE").AsResource().SetResource(srv);
            DeviceManager.Context.InputAssembler.SetVertexBuffers(0, new VertexBufferBinding(VertexBuffer, SpriteVertexLayout.SizeInBytes, 0));
            DeviceManager.Context.InputAssembler.InputLayout = VertexInputLayout;
            DeviceManager.Context.InputAssembler.PrimitiveTopology = PrimitiveTopology.TriangleList;
            DeviceManager.Context.OutputMerger.BlendState = state;
            DeviceManager.Context.Rasterizer.SetViewports(spriteViewport);
            renderPass.Apply(DeviceManager.Context);
            DeviceManager.Context.Draw(6, 0);
            DeviceManager.Context.Rasterizer.SetViewports(lastViewports);
            DeviceManager.Context.OutputMerger.BlendState = lastBlendState;
            srv.Dispose();
            MutexD3D11.Release(0);
        }

        public void Dispose()
        {
            if (BatchDisposing != null) BatchDisposing(this, new EventArgs());
            if (TextureD3D10 != null && !TextureD3D10.Disposed) TextureD3D10.Dispose();
            if (TextureD3D11 != null && !TextureD3D11.Disposed) TextureD3D11.Dispose();
            if (MutexD3D10 != null && !MutexD3D10.Disposed) MutexD3D10.Dispose();
            if (MutexD3D11 != null && !MutexD3D11.Disposed) MutexD3D11.Dispose();
            if (DWRenderTarget != null && !DWRenderTarget.Disposed) DWRenderTarget.Dispose();
            if (VertexBuffer != null && !VertexBuffer.Disposed) VertexBuffer.Dispose();
            if (VertexInputLayout != null && !VertexInputLayout.Disposed) VertexInputLayout.Dispose();
            if (SpriteEffect != null && !SpriteEffect.Disposed) SpriteEffect.Dispose();
            if (sampler != null && !sampler.Disposed) sampler.Dispose();
            //以下の二つはRenderContextがDisposeされるときに呼ばれないとバグが出る。(画面真っ黒)
            //これは同じ種類のブレンドステートがDisposeされてしまうからだと予測できる。
            //このため、Dispose予定リストに含めておき、RenderContextの破棄時に処理をする
            context.Disposables.Add(TransParentBlendState);
            context.Disposables.Add(state);
        }


        public Vector2 TextureSize { get;private set; }

        /// <summary>
        /// 指定した単色ブラシを取得します。
        /// </summary>
        /// <param name="color"></param>
        /// <returns></returns>
        public D2DSpriteSolidColorBrush CreateSolidColorBrush(Color color)
        {
            return new D2DSpriteSolidColorBrush(this,color);
        }

        public D2DSpriteTextformat CreateTextformat(string fontFamiry, int size=15, FontWeight weight=FontWeight.Normal,
            FontStyle style=FontStyle.Normal, FontStretch stretch=FontStretch.Normal, string locale="ja-jp")
        {
           return new D2DSpriteTextformat(this,fontFamiry,size,weight,style,stretch,locale);
        }

        public D2DSpriteBitmap CreateBitmap(string fileName)
        {
            return new D2DSpriteBitmap(this,File.OpenRead(fileName));
        }

        public D2DSpriteBitmap CreateBitmap(Stream fs)
        {
            return new D2DSpriteBitmap(this,fs);
        }

        public D2DSpriteBitmapBrush CreateBitmapBrush(string fileName,BitmapBrushProperties bbp=new BitmapBrushProperties())
        {
            return new D2DSpriteBitmapBrush(this,CreateBitmap(fileName),bbp);
        }

        public D2DSpriteBitmapBrush CreateBitmapBrush(Stream fileStream, BitmapBrushProperties bbp = new BitmapBrushProperties())
        {
            return new D2DSpriteBitmapBrush(this, CreateBitmap(fileStream), bbp);
        }

        public D2DSpriteLinearGradientBrush CreateLinearGradientBrush(D2DSpriteGradientStopCollection collection,
            LinearGradientBrushProperties gradient)
        {
            return new D2DSpriteLinearGradientBrush(this,collection,gradient);
        }

        public D2DSpriteGradientStopCollection CreateGradientStopCollection(GradientStop[] stops)
        {
            return new D2DSpriteGradientStopCollection(this,stops,Gamma.Linear,ExtendMode.Mirror);
        }

        public D2DSpriteRadialGradientBrush CreateRadialGradientBrush(D2DSpriteGradientStopCollection collection,
            RadialGradientBrushProperties r)
        {
            return new D2DSpriteRadialGradientBrush(this,collection.GradientStopCollection,r);
        }
    }
}
