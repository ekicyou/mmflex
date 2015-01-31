using System;
using System.Collections.Generic;
using MMF.MME.Script;
using MMF.Model;
using SlimDX.Direct3D11;

namespace MMF.MME
{
    /// <summary>
    ///     MME形式のシェーダーのテクニックを管理するクラス
    /// </summary>
    public class MMEEffectTechnique
    {
        /// <summary>
        ///     コンストラクタ
        /// </summary>
        /// <param name="manager"></param>
        /// <param name="technique"></param>
        /// <param name="subsetCount"></param>
        /// <param name="context"></param>
        public MMEEffectTechnique(MMEEffectManager manager, EffectTechnique technique, int subsetCount, 
            RenderContext context)
        {
            
            Subset = new HashSet<int>();
            Passes = new Dictionary<string, MMEEffectPass>();
            if (!technique.IsValid)
                throw new InvalidMMEEffectShaderException(string.Format("テクニック「{0}」の検証に失敗しました。",
                    technique.Description.Name));
            //MMDPassの読み込み
            string mmdpass = EffectParseHelper.getAnnotationString(technique, "MMDPass");
            if (String.IsNullOrWhiteSpace(mmdpass))
            {
                MMDPassAnnotation = MMEEffectPassType.Object;
            }
            else
            {
                mmdpass = mmdpass.ToLower();
                switch (mmdpass)
                {
                    case "object":
                        MMDPassAnnotation = MMEEffectPassType.Object;
                        break;
                    case "object_ss":
                        MMDPassAnnotation = MMEEffectPassType.Object_SelfShadow;
                        break;
                    case "zplot":
                        MMDPassAnnotation = MMEEffectPassType.ZPlot;
                        break;
                    case "shadow":
                        MMDPassAnnotation = MMEEffectPassType.Shadow;
                        break;
                    case "edge":
                        MMDPassAnnotation = MMEEffectPassType.Edge;
                        break;
                    default:
                        throw new InvalidOperationException("予期しない識別子");
                }
            }
            //UseTextureの読み込み
            UseTexture = EffectParseHelper.getAnnotationBoolean(technique, "UseTexture");
            UseSphereMap = EffectParseHelper.getAnnotationBoolean(technique, "UseSphereMap");
            UseToon = EffectParseHelper.getAnnotationBoolean(technique, "UseToon");
            UseSelfShadow = EffectParseHelper.getAnnotationBoolean(technique, "UseSelfShadow");
            MulSphere = EffectParseHelper.getAnnotationBoolean(technique, "MulSphere");
            GetSubsets(technique, subsetCount);
            EffectVariable rawScript = EffectParseHelper.getAnnotation(technique, "Script", "string");
            for (int i = 0; i < technique.Description.PassCount; i++)
            {
                EffectPass pass = technique.GetPassByIndex(i);
                Passes.Add(pass.Description.Name,new MMEEffectPass(context, manager, pass));
            }
            if (rawScript != null)
            {
                ScriptRuntime = new ScriptRuntime(rawScript.AsString().GetString(), context, manager, this);
            }
            else
            {
                ScriptRuntime = new ScriptRuntime("", context, manager, this);
            }
        }

        /// <summary>
        ///     このクラスが描画するサブセット番号
        /// </summary>
        public HashSet<int> Subset { get; private set; }

        /// <summary>
        ///     このテクニックに利用されるパス
        /// </summary>
        public Dictionary<string,MMEEffectPass> Passes { get; private set; }

        public ScriptRuntime ScriptRuntime { get; private set; }

        /// <summary>
        ///     MMDPassのタイプ
        /// </summary>
        public MMEEffectPassType MMDPassAnnotation { get; private set; }

        /// <summary>
        ///     スフィアマップを使うかどうか
        /// </summary>
        public ExtendedBoolean UseSphereMap { get; private set; }

        /// <summary>
        ///     テクスチャを使うかどうか
        /// </summary>
        public ExtendedBoolean UseTexture { get; private set; }

        /// <summary>
        ///     トゥーンを使うかどうか
        /// </summary>
        public ExtendedBoolean UseToon { get; private set; }

        /// <summary>
        ///     セルフシャドウを使うかどうか(MMM仕様)
        /// </summary>
        public ExtendedBoolean UseSelfShadow { get; private set; }

        public ExtendedBoolean MulSphere { get; private set; }

        private void GetSubsets(EffectTechnique technique, int subsetCount)
        {
            string subset = EffectParseHelper.getAnnotationString(technique, "Subset");
            //Subsetの解析
            if (string.IsNullOrWhiteSpace(subset))
            {
                for (int i = 0; i <= subsetCount; i++) //指定しない場合は全てがレンダリング対象のサブセットとなる
                {
                    Subset.Add(i);
                }
            }
            else
            {
                string[] chunks = subset.Split(','); //,でサブセットアノテーションを分割
                foreach (string chunk in chunks)
                {
                    if (chunk.IndexOf('-') == -1) //-がない場合はそれが単体であると認識する
                    {
                        int value = 0;
                        if (int.TryParse(chunk, out value))
                        {
                            Subset.Add(value);
                        }
                        else
                        {
                            throw new InvalidMMEEffectShaderException(
                                string.Format("テクニック「{0}」のサブセット解析中にエラーが発生しました。「{1}」中の「{2}」は認識されません。",
                                    technique.Description.Name, subset, chunk));
                        }
                    }
                    else
                    {
                        string[] regions = chunk.Split('-'); //-がある場合は範囲指定と認識する。
                        if (regions.Length > 2)
                            throw new InvalidMMEEffectShaderException(
                                string.Format("テクニック「{0}」のサブセット解析中にエラーが発生しました。「{1}」中の「{2}」には\"-\"が2つ以上存在します。",
                                    technique.Description.Name, subset, chunk));
                        if (string.IsNullOrWhiteSpace(regions[1])) //この場合、X-の形だと認識される。
                        {
                            int value = 0;
                            if (int.TryParse(regions[0], out value))
                            {
                                for (int i = value; i <= subsetCount; i++)
                                {
                                    Subset.Add(i);
                                }
                            }
                            else
                            {
                                throw new InvalidMMEEffectShaderException(
                                    string.Format("テクニック「{0}」のサブセット解析中にエラーが発生しました。「{1}」中の「{2}」の「{3}」は認識されません。",
                                        technique.Description.Name, subset, chunk, regions[0]));
                            }
                        }
                        else //この場合X-Yの形式だと認識される
                        {
                            int value1 = 0;
                            int value2 = 0;
                            if (int.TryParse(regions[0], out value1) && int.TryParse(regions[1], out value2))
                            {
                                for (int i = value1; i <= value2; i++)
                                {
                                    Subset.Add(i);
                                }
                            }
                            else
                            {
                                throw new InvalidMMEEffectShaderException(
                                    string.Format(
                                        "テクニック「{0}」のサブセット解析中にエラーが発生しました。「{1}」中の「{2}」の「{3}」もしくは「{4}」は認識されません。",
                                        technique.Description.Name, subset, chunk, regions[0], regions[1]));
                            }
                        }
                    }
                }
            }
        }

        public void ExecuteTechnique(DeviceContext context,Action<ISubset> drawAction,ISubset ipmxSubset)
        {
            if (string.IsNullOrWhiteSpace(ScriptRuntime.ScriptCode))
            {
                foreach (MMEEffectPass pass in Passes.Values)
                {
                    pass.Pass.Apply(context);
                    drawAction(ipmxSubset);
                }
            }
            else//スクリプトが存在する場合は処理をスクリプトランタイムに任せる
            {
                ScriptRuntime.Execute(drawAction, ipmxSubset);
            }
        }

        public static bool CheckExtebdedBoolean(ExtendedBoolean teqValue, bool subsetValue)
        {
            if (subsetValue)
            {
                return teqValue != ExtendedBoolean.Disable;
            }
            return teqValue != ExtendedBoolean.Enable;
        }
    }
}