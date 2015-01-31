using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using MMF.DeviceManager;
using MMF.MME.Includer;
using MMF.MME.VariableSubscriber;
using MMF.MME.VariableSubscriber.ConstantSubscriber;
using MMF.MME.VariableSubscriber.ControlInfoSubscriber;
using MMF.MME.VariableSubscriber.MaterialSubscriber;
using MMF.MME.VariableSubscriber.MatrixSubscriber;
using MMF.MME.VariableSubscriber.MouseSubscriber;
using MMF.MME.VariableSubscriber.PeculiarValueSubscriber;
using MMF.MME.VariableSubscriber.ScreenInfoSubscriber;
using MMF.MME.VariableSubscriber.TextureSubscriber;
using MMF.MME.VariableSubscriber.TimeSubscriber;
using MMF.MME.VariableSubscriber.WorldInfoSubscriber;
using MMF.Model;
using MMF.Model.PMX;
using MMF.Utility;
using SlimDX.D3DCompiler;
using SlimDX.Direct3D11;

namespace MMF.MME
{
    /// <summary>
    ///     MMEのエフェクトを管理するクラス
    /// </summary>
    public class MMEEffectManager : IDisposable
    {
        private static string applicationDefinition = "MMFApp";

        /// <summary>
        ///     利用対象のモデル
        /// </summary>
        private readonly IDrawable model;

        private string fileName;

        /// <summary>
        ///     コンストラクタ
        /// </summary>
        /// <param name="fileName"></param>
        /// <param name="context"></param>
        /// <param name="effect">解釈対象のエフェクト</param>
        /// <param name="model">使用対象のモデル</param>
        /// <param name="loader"></param>
        private MMEEffectManager(string fileName, RenderContext context, SlimDX.Direct3D11.Effect effect, IDrawable model,ISubresourceLoader loader)
        {
            this.fileName = fileName;
            SubresourceLoader = loader;
            if (!fileName.Equals(MMFDefaultShaderResourcePath))
            {
                //ファイルパスがデフォルトのシェーダーと等しくない時は、デフォルトのシェーダーも読み込んでおく。
                DefaultShader = LoadFromResource(MMFDefaultShaderResourcePath, model, context,new BasicSubresourceLoader("Shader"));
            }
            else
            {
                DefaultShader = this; //等しい時は自身がデフォルトのシェーダーと等しい。
            }
            Context = context;
            EffectFile = effect;
            EffectInfo = new MMEEffectInfo(effect);
            ActiveSubscriberByMaterial = new Dictionary<EffectVariable, SubscriberBase>();
            ActiveSubscriberByModel = new Dictionary<EffectVariable, SubscriberBase>();
            ActivePeculiarSubscriber = new Dictionary<EffectVariable, PeculiarValueSubscriberBase>();
            Techniques = new List<MMEEffectTechnique>();
            RenderColorTargetViewes=new Dictionary<string, RenderTargetView>();
            RenderDepthStencilTargets=new Dictionary<string, DepthStencilView>();
            this.model = model;
            //グローバル変数の登録
            int valCount = effect.Description.GlobalVariableCount;
            for (int i = 0; i < valCount; i++)
            {
                string semantic = Regex.Replace(effect.GetVariableByIndex(i).Description.Semantic,"[0-9]","");
                string semanticIndexStr = Regex.Replace(effect.GetVariableByIndex(i).Description.Semantic, "[^0-9]", "");
                int semanticIndex =string.IsNullOrEmpty(semanticIndexStr)?0:int.Parse(semanticIndexStr);
                string typeName = effect.GetVariableByIndex(i).GetVariableType().Description.TypeName.ToLower();
                semantic = semantic.ToUpper(); //小文字でもいいようにするため大文字に全て変換
                if (EffectSubscriber.ContainsKey(semantic))
                {//通常はセマンティクスに応じて登録
                    SubscriberBase subs = EffectSubscriber[semantic];
                    EffectVariable variable = effect.GetVariableByIndex(i);
                    subs.CheckType(variable);
                    if (subs.UpdateTiming == UpdateBy.Material)
                    {
                        ActiveSubscriberByMaterial.Add(variable, subs.GetSubscriberInstance(variable, context, this, semanticIndex));
                    }
                    else
                    {
                        ActiveSubscriberByModel.Add(variable, subs.GetSubscriberInstance(variable, context, this, semanticIndex));
                    }
                }else if (typeName.Equals("texture") || typeName.Equals("texture2d") || typeName.Equals("texture3d") ||
                          typeName.Equals("texturecube"))//テクスチャのみ例外で、変数型に応じて登録する
                {
                    SubscriberBase subs=new TextureSubscriber();
                    EffectVariable variable = effect.GetVariableByIndex(i);
                    subs.CheckType(variable);
                    ActiveSubscriberByModel.Add(variable,subs.GetSubscriberInstance(variable,context,this, semanticIndex));
                }
                else//特殊変数は変数名に応じて登録する
                {
                    string name = effect.GetVariableByIndex(i).Description.Name;
                    name = name.ToLower();
                    if (PeculiarEffectSubscriber.ContainsKey(name))
                    {
                        ActivePeculiarSubscriber.Add(effect.GetVariableByIndex(i), PeculiarEffectSubscriber[name]);
                    }
                }
            }
            //定数バッファの登録
            valCount = effect.Description.ConstantBufferCount;
            for (int i = 0; i < valCount; i++)
            {
                string name = effect.GetConstantBufferByIndex(i).Description.Name;
                name = name.ToUpper();
                if (EffectSubscriber.ContainsKey(name))
                {
                    SubscriberBase subs = EffectSubscriber[name]; //定数バッファにはセマンティクスが付けられないため、変数名で代用
                    EffectConstantBuffer variable = effect.GetConstantBufferByIndex(i);
                    subs.CheckType(variable);
                    if (subs.UpdateTiming == UpdateBy.Material)
                    {
                        ActiveSubscriberByMaterial.Add(variable, subs.GetSubscriberInstance(variable, context, this, 0));
                    }
                    else
                    {
                        ActiveSubscriberByModel.Add(variable, subs.GetSubscriberInstance(variable, context, this, 0));
                    }
                }
            }

            int subsetCount = model is ISubsetDivided ? ((ISubsetDivided)model).SubsetCount : 1;
            foreach (EffectTechnique t in EffectInfo.SortedTechnique)//MMEEffectTechniqueもソートしておく
            {
                //テクニックをすべて読みだす
                Techniques.Add(new MMEEffectTechnique(this, t, subsetCount, context));
            }
        }

        #region 静的な部分

        /// <summary>
        ///     エフェクトに定義するMMFの定数
        /// </summary>
        public const string MMFDefinition = "MMF";

        /// <summary>
        ///     デフォルトのシェーダーのファイルパス
        /// </summary>
        public const string MMFDefaultShaderResourcePath = "MMF.Resource.Shader.DefaultShader.fx";

        /// <summary>
        ///     エフェクトへの変数登録クラスのリスト
        /// </summary>
        public static EffectSubscriberDictionary EffectSubscriber { get; private set; }

        public static PeculiarEffectSubscriberDictionary PeculiarEffectSubscriber { get; private set; }

        /// <summary>
        ///     エフェクトに定義するマクロのリスト
        /// </summary>
        public static List<ShaderMacro> EffectMacros { get; private set; }

        /// <summary>
        ///     エフェクトに含めるIncludeリスト
        /// </summary>
        public static Include EffectInclude { get; private set; }

        /// <summary>
        ///     エフェクトに定義するアプリケーションの定数
        /// </summary>
        public static string ApplicationDefinition
        {
            get { return applicationDefinition; }
            set
            {
                if (applicationDefinition != value)
                {
//前と違う値がセットされた場合、マクロのリストをすり替える
                    EffectMacros.Remove(new ShaderMacro(applicationDefinition));
                    applicationDefinition = value;
                    EffectMacros.Add(new ShaderMacro(value));
                }
            }
        }

        #endregion

        private RenderContext Context { get; set; }

        public MMEEffectInfo EffectInfo { get; private set; }
        
        /// <summary>
        /// モデルごとに登録すべき変数の登録インスタンス
        /// </summary>
        public Dictionary<EffectVariable, SubscriberBase> ActiveSubscriberByModel { get; private set; }

        /// <summary>
        /// マテリアルごとに登録すべき変数の登録インスタンス
        /// </summary>
        public Dictionary<EffectVariable, SubscriberBase> ActiveSubscriberByMaterial { get; private set; }

        /// <summary>
        /// 変数名で判定するタイプの特殊変数の登録インスタンス
        /// </summary>
        public Dictionary<EffectVariable, PeculiarValueSubscriberBase> ActivePeculiarSubscriber { get; private set; }

        public Dictionary<string,RenderTargetView> RenderColorTargetViewes { get; private set; } 

        public Dictionary<string,DepthStencilView> RenderDepthStencilTargets { get; private set; } 
        /// <summary>
        ///     エフェクトに記述されているテクニックのリスト
        /// </summary>
        public List<MMEEffectTechnique> Techniques { get; private set; }

        /// <summary>
        /// テクスチャなどのパスの解決に利用するローダー
        /// </summary>
        public ISubresourceLoader SubresourceLoader { get; private set; }

        /// <summary>
        ///     利用するエフェクトファイル本体
        /// </summary>
        private SlimDX.Direct3D11.Effect effect { get; set; }

        /// <summary>
        ///     デフォルトのシェーダー
        /// </summary>
        public MMEEffectManager DefaultShader { get; private set; }


        /// <summary>
        ///     エフェクトファイル
        /// </summary>
        public SlimDX.Direct3D11.Effect EffectFile
        {
            get { return effect; }
            private set { effect = value; }
        }

        /// <summary>
        ///     破棄
        /// </summary>
        public void Dispose()
        {
            EffectFile.Dispose();
            foreach (KeyValuePair<EffectVariable, SubscriberBase> subscriberAnnotation in ActiveSubscriberByMaterial)
            {
                if (subscriberAnnotation.Value is IDisposable)
                {
                    IDisposable disposeTarget = (IDisposable) subscriberAnnotation.Value;
                    if (disposeTarget != null) disposeTarget.Dispose();
                }
            }
            foreach (KeyValuePair<EffectVariable, SubscriberBase> subscriberBase in ActiveSubscriberByModel)
            {
                if (subscriberBase.Value is IDisposable)
                {
                    IDisposable disposeTarget = (IDisposable)subscriberBase.Value;
                    if (disposeTarget != null) disposeTarget.Dispose();
                }
            }
        }

        internal static void IniatializeMMEEffectManager(IDeviceManager deviceManager)
        {
            //エフェクトへの変数登録クラスの登録
            EffectSubscriber = new EffectSubscriberDictionary
            {
                new WorldMatrixSubscriber(),
                new ProjectionMatrixSubscriber(),
                new ViewMatrixSubscriber(),
                new WorldInverseMatrixSubscriber(),
                new WorldTransposeMatrixSubscriber(),
                new WorldInverseTransposeMatrixSubscriber(),
                new ViewInverseMatrixSubscriber(),
                new ViewTransposeMatrixSubscriber(),
                new ViewInverseTransposeMatrixSubsriber(),
                new ProjectionInverseMatrixSubscriber(),
                new ProjectionTransposeMatrixSubscriber(),
                new ProjectionInverseTransposeMatrixSubscriber(),
                new WorldViewMatrixSubscriber(),
                new WorldViewInverseMatrixSubscriber(),
                new WorldViewTransposeMatrixSubscriber(),
                new ViewProjectionMatrixSubscriber(),
                new ViewProjectionInverseMatrixSubscriber(),
                new ViewProjectionTransposeMatrixSubscriber(),
                new ViewProjectionInverseTransposeMatrixSubscriber(),
                new WorldViewProjectionMatrixSubscriber(),
                new WorldViewProjectionInverseMatrixSubscriber(),
                new WorldViewProjectionTransposeMatrixSubscriber(),
                new WorldViewProjectionInverseTransposeMatrixSubscriber(),
                //マテリアル
                new DiffuseVectorSubscriber(),
                new AmbientVectorSubscriber(),
                new SpecularVectorSubscriber(),
                new SpecularPowerSubscriber(),
                new ToonVectorSubscriber(),
                new EdgeVectorSubscriber(),
                new GroundShadowColorVectorSubscriber(),
                new MaterialTextureSubscriber(),
                new MaterialSphereMapSubscriber(),
                new MaterialToonTextureSubscriber(),
                new AddingTextureSubscriber(),
                new MultiplyingTextureSubscriber(),
                new AddingSphereTextureSubscriber(),
                new MultiplyingSphereTextureSubscriber(),
                new EdgeThicknessSubscriber(),
                //位置/方向
                new PositionSubscriber(),
                new DirectionSubscriber(),
                //時間
                new TimeSubScriber(),
                new ElapsedTimeSubScriber(),
                //マウス
                new MousePositionSubscriber(),
                new LeftMouseDownSubscriber(),
                new MiddleMouseDownSubscriber(),
                new RightMouseDownSubscriber(),
                //スクリーン情報
                new ViewPortPixelSizeScriber(),
                //定数バッファ
                new BasicMaterialConstantSubscriber(),
                new FullMaterialConstantSubscriber(),
                //コントロールオブジェクト
                new ControlObjectSubscriber(),
                new RenderColorTargetSubscriber(),
                new RenderDepthStencilTargetSubscriber(),
            };
            PeculiarEffectSubscriber = new PeculiarEffectSubscriberDictionary
            {
                new OpAddSubscriber(),
                new ParthfSubscriber(),
                new SpAddSubscriber(),
                new SubsetCountSubscriber(),
                new TranspSubscriber(),
                new Use_SpheremapSubscriber(),
                new Use_TextureSubscriber(),
                new Use_ToonSubscriber(),
                new VertexCountSubscriber()
            };
            //エフェクトのコンパイル用マクロの初期化
            EffectMacros = new List<ShaderMacro>();
            EffectMacros.Add(new ShaderMacro(MMFDefinition)); //定数MMFを定義
            EffectMacros.Add(new ShaderMacro(ApplicationDefinition)); //アプリケーションの定数を定義
            EffectMacros.Add(new ShaderMacro("MMM_LightCount", "3"));
            if (deviceManager.DeviceFeatureLevel == FeatureLevel.Level_11_0)
            {
                EffectMacros.Add(new ShaderMacro("DX_LEVEL_11_0"));
            }
            else
            {
                EffectMacros.Add(new ShaderMacro("DX_LEVEL_10_1"));
            }
            EffectInclude = new BasicEffectIncluder();
        }

        /// <summary>
        ///     すべての必要な変数をエフェクトに対して割り当てる
        /// </summary>
        /// <param name="matricies">利用するワールド行列</param>
        public void ApplyAllMatrixVariables()
        {
            SubscribeArgument argument = new SubscribeArgument(model, Context);
            foreach (KeyValuePair<EffectVariable, SubscriberBase> subscriberBase in ActiveSubscriberByModel)
            {
                subscriberBase.Value.Subscribe(subscriberBase.Key, argument);
            }
        }

        /// <summary>
        ///     全ての必要なマテリアルの変数を割り当てる
        /// </summary>
        /// <param name="info">利用するマテリアル情報</param>
        public void ApplyAllMaterialVariables(MaterialInfo info)
        {
            
            SubscribeArgument argument = new SubscribeArgument(info, model, Context);
            foreach (KeyValuePair<EffectVariable, SubscriberBase> item in ActiveSubscriberByMaterial)
            {
                item.Value.Subscribe(item.Key, argument);
            }
            foreach (
                KeyValuePair<EffectVariable, PeculiarValueSubscriberBase> peculiarValueSubscriberBase in ActivePeculiarSubscriber)
            {
                peculiarValueSubscriberBase.Value.Subscribe(peculiarValueSubscriberBase.Key,
                    argument);
            }
        }

        /// <summary>
        ///     エフェクトパスを割り当てる
        /// </summary>
        /// <param name="ipmxSubset"></param>
        public void ApplyEffectPass(ISubset ipmxSubset, MMEEffectPassType passType, Action<ISubset> drawAction)
        {
            if(ipmxSubset.MaterialInfo.DiffuseColor.W==0)return;
            //両面描画かどうかに応じてカリングの値を切り替える
            if (ipmxSubset.DoCulling)
            {
                Context.DeviceManager.Context.Rasterizer.State = Context.CullingRasterizerState;
            }
            else
            {
                Context.DeviceManager.Context.Rasterizer.State = Context.NonCullingRasterizerState;
            }
            //使用するtechniqueを検索する
            MMEEffectTechnique[] techniques = (from teq in Techniques
                where
                    teq.Subset.Contains(ipmxSubset.SubsetId) && teq.MMDPassAnnotation == passType &&
                    MMEEffectTechnique.CheckExtebdedBoolean(teq.UseToon, ipmxSubset.MaterialInfo.IsToonUsed) &&
                    MMEEffectTechnique.CheckExtebdedBoolean(teq.UseTexture, ipmxSubset.MaterialInfo.MaterialTexture != null) &&
                    MMEEffectTechnique.CheckExtebdedBoolean(teq.UseSphereMap,
                        ipmxSubset.MaterialInfo.MaterialSphereMap != null) && MMEEffectTechnique.CheckExtebdedBoolean(teq.MulSphere, ipmxSubset.MaterialInfo.SphereMode == MMDFileParser.PMXModelParser.SphereMode.Multiply)
                select teq).ToArray();
            foreach (MMEEffectTechnique technique in techniques)
            {
                technique.ExecuteTechnique(Context.DeviceManager.Context,drawAction,ipmxSubset);
                return;
            }
        }

        /// <summary>
        ///     MMEのエフェクトとして読み込む
        /// </summary>
        /// <param name="str">エフェクトのパス</param>
        /// <param name="model">使用対象のモデル</param>
        /// <param name="context"></param>
        /// <param name="loader"></param>
        /// <param name="device">デバイス</param>
        /// <returns>MME仕様のエフェクト</returns>
        public static MMEEffectManager Load(string str, IDrawable model, RenderContext context,ISubresourceLoader loader)
        {
            return new MMEEffectManager(str, context, CGHelper.CreateEffectFx5(str, context.DeviceManager.Device), model, loader);
        }

        /// <summary>
        ///     MMEのエフェクトとして読み込む
        /// </summary>
        /// <param name="str">エフェクトのパス</param>
        /// <param name="model">使用対象のモデル</param>
        /// <param name="context"></param>
        /// <param name="loader"></param>
        /// <param name="device">デバイス</param>
        /// <returns>MME仕様のエフェクト</returns>
        internal static MMEEffectManager LoadFromResource(string str, IDrawable model, RenderContext context,ISubresourceLoader loader)
        {
            SlimDX.Direct3D11.Effect effect = CGHelper.CreateEffectFx5FromResource(str, context.DeviceManager.Device);
            return new MMEEffectManager(str, context, effect, model,loader);
        }
    }
}