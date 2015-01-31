using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using MMF.Utility;
using SlimDX;
using SlimDX.Direct3D11;
using Buffer = SlimDX.Direct3D11.Buffer;

namespace MMF.Sprite
{
    /// <summary>
    /// 2D表示用のクラス
    /// 速度は保証できない
    /// </summary>
    public class GDISpriteBatch:IDisposable
    {
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

        /// <summary>
        /// スプライトのテクスチャの描画先頂点バッファ
        /// </summary>
        private Buffer VertexBuffer { get;set; }

        /// <summary>
        /// スプライトの入力レイアウト
        /// </summary>
        private InputLayout VertexInputLayout { get; set; }

        /// <summary>
        /// スプライトの描画に利用するエフェクト
        /// </summary>
        public Effect SpriteEffect { get; private set; }

        /// <summary>
        /// スプライトの描画に利用するパス
        /// </summary>
        private EffectPass renderPass { get; set; }

        /// <summary>
        /// スプライトに描画されるテクスチャ
        /// </summary>
        public ShaderResourceView SpriteTexture { get; private set; }

        /// <summary>
        /// テクスチャのサイズ
        /// </summary>
        public Vector2 TextureSize { get; private set; }

        /// <summary>
        /// 透過色
        /// </summary>
        public Vector3 TransparentColor { get; private set; }

        /// <summary>
        /// スプライトに結び付けられたBitmap
        /// </summary>
        private Bitmap mapedBitmap { get; set; }

        /// <summary>
        /// スプライトに結び付けられたGraphic
        /// 編集後はNeedRedrawをtrueにする。
        /// </summary>
        public  Graphics mapedGraphic { get;private set; }

        /// <summary>
        /// スプライトのテクスチャの再描画の必要かどうか
        /// </summary>
        public bool NeedRedraw { get; set; }

        /// <summary>
        /// スプライトのテクスチャ用のサンプラ
        /// </summary>
        private SamplerState sampler { get; set; }

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="context">レンダーコンテキスト</param>
        /// <param name="width">描画に利用するテクスチャの解像度の幅</param>
        /// <param name="height">描画に利用するテクスチャの解像度の高さ</param>
        public GDISpriteBatch(RenderContext context,int width,int height)
        {
            this.Context = context;
            Resize(width,height);
            SpriteEffect = CGHelper.CreateEffectFx5("Shader\\sprite.fx", context.DeviceManager.Device);
            renderPass = SpriteEffect.GetTechniqueByIndex(0).GetPassByIndex(0);
            VertexInputLayout = new InputLayout(context.DeviceManager.Device, SpriteEffect.GetTechniqueByIndex(0).GetPassByIndex(0).Description.Signature, SpriteVertexLayout.InputElements);
            SamplerDescription desc=new SamplerDescription();
            desc.Filter = Filter.MinMagMipLinear;
            desc.AddressU=TextureAddressMode.Wrap;
            desc.AddressV = TextureAddressMode.Wrap;
            desc.AddressW = TextureAddressMode.Wrap;
            sampler = SamplerState.FromDescription(context.DeviceManager.Device, desc);
            mapedGraphic = Graphics.FromImage(mapedBitmap);
            BlendStateDescription blendDesc=new BlendStateDescription();
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
            ViewMatrix = Matrix.LookAtLH(new Vector3(0, 0, -1), new Vector3(0, 0, 0), new Vector3(0, 1, 0));
            state = BlendState.FromDescription(context.DeviceManager.Device, blendDesc);
            NeedRedraw = true;
            Update();
        }

        /// <summary>
        /// テクスチャのサイズを変更する
        /// </summary>
        /// <param name="width">幅</param>
        /// <param name="height">高さ</param>
        public void Resize(int width, int height)
        {
            TextureSize=new Vector2(width,height);
            float w = width/2f, h = height/2f;
            List<byte> vertexBytes = new List<byte>();
            TransparentColor = new Vector3(0, 0, 0);
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
                    SizeInBytes = (int) ds.Length
                };
                VertexBuffer = new Buffer(Context.DeviceManager.Device, ds, bufDesc);
            }
            SpriteProjectionMatrix = Matrix.OrthoLH(width, height, 0, 100);
            spriteViewport = new Viewport()
            {
                Width = width,
                Height = height,
                MaxZ = 1
            };
            mapedBitmap=new Bitmap(width,height);
            if(mapedGraphic!=null)mapedGraphic.Dispose();
            mapedGraphic = Graphics.FromImage(mapedBitmap);
        }

        /// <summary>
        /// テクスチャを更新する必要が有る場合更新する
        /// レンダリングと同一タイミングで呼び出すこと。
        /// </summary>
        public void Update()
        {
            if (NeedRedraw)
            {
                using (MemoryStream ms = new MemoryStream())
                {
                        lock (mapedBitmap)
                        {
                            mapedBitmap.Save(ms, ImageFormat.Tiff);
                        }
                        ms.Seek(0, SeekOrigin.Begin);
                    try
                    {
                        if(SpriteTexture!=null)
                        lock (SpriteTexture)
                        {
                            SpriteTexture.Dispose();
                            SpriteTexture = ShaderResourceView.FromStream(Context.DeviceManager.Device, ms, (int)ms.Length);
                        }
                        else
                        {
                            SpriteTexture = ShaderResourceView.FromStream(Context.DeviceManager.Device, ms, (int)ms.Length);
                        }
                    }
                    catch (Direct3D11Exception)
                    {
                        //終了時になぜかスローされる。とりあえず無視
                    }
                }
                NeedRedraw = false;
            }
        }

        /// <summary>
        /// スプライトを描画する
        /// </summary>
        public void Draw()
        {
            BlendState lastBlendState = Context.DeviceManager.Context.OutputMerger.BlendState;
            Viewport[] lastViewports = Context.DeviceManager.Context.Rasterizer.GetViewports();
            //WVP行列をエフェクトに渡す
            SpriteEffect.GetVariableBySemantic("WORLDVIEWPROJECTION")
                .AsMatrix()
                .SetMatrix(ViewMatrix*SpriteProjectionMatrix);
            SpriteEffect.GetVariableBySemantic("TRANSCOLOR").AsVector().Set(TransparentColor);
            SpriteEffect.GetVariableBySemantic("SPRITETEXTURE").AsResource().SetResource(SpriteTexture);
            Context.DeviceManager.Context.InputAssembler.SetVertexBuffers(0,new VertexBufferBinding(VertexBuffer,SpriteVertexLayout.SizeInBytes,0));
            Context.DeviceManager.Context.InputAssembler.InputLayout = VertexInputLayout;
            Context.DeviceManager.Context.InputAssembler.PrimitiveTopology = PrimitiveTopology.TriangleList;
            Context.DeviceManager.Context.OutputMerger.BlendState = state;
            Context.DeviceManager.Context.Rasterizer.SetViewports(spriteViewport);
            renderPass.Apply(Context.DeviceManager.Context);
            Context.DeviceManager.Context.Draw(6,0);
            Context.DeviceManager.Context.Rasterizer.SetViewports(lastViewports);
            Context.DeviceManager.Context.OutputMerger.BlendState = lastBlendState;
        }



        private RenderContext Context { get;set; }
        public void Dispose()
        {
            if (VertexBuffer != null && !VertexBuffer.Disposed) VertexBuffer.Dispose();
            if (VertexInputLayout != null && !VertexInputLayout.Disposed) VertexInputLayout.Dispose();
            if (SpriteEffect != null && !SpriteEffect.Disposed) SpriteEffect.Dispose();
            if (SpriteTexture != null && !SpriteTexture.Disposed) SpriteTexture.Dispose();
            if (mapedBitmap != null) mapedBitmap.Dispose();
            if (mapedGraphic != null) mapedGraphic.Dispose();
        }
    }
}
