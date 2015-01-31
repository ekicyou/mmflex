using System;
using System.Linq;
using System.Text;
using SlimDX.Direct3D11;

namespace MMF.MME
{
    /// <summary>
    ///     エフェクトのパースのヘルパークラス
    /// </summary>
    public static class EffectParseHelper
    {
        /// <summary>
        ///     アノテーション名の大文字小文字を無視して取得する
        /// </summary>
        /// <param name="variable">取得対象のエフェクト変数</param>
        /// <param name="target">アノテーション名</param>
        /// <param name="typeName">期待する型</param>
        /// <returns>アノテーション</returns>
        public static EffectVariable getAnnotation(EffectVariable variable, string target, string typeName)
        {
            string name = target.ToLower();
            string[] valid = name.Split('/');
            for (int i = 0; i < variable.Description.AnnotationCount; i++)
            {
                EffectVariable val = variable.GetAnnotationByIndex(i);
                string typeString = val.Description.Name.ToLower();
                if (typeString== name)
                {
                    if (
                        !valid.Contains(typeString)&&!String.IsNullOrWhiteSpace(typeString))
                    {
                        throw new InvalidMMEEffectShaderException(
                            string.Format(
                                "変数「{0} {1}:{2}」に適用されたアノテーション「{3} {4}」はアノテーションの型が正しくありません。期待した型は{5}でした。",
                                variable.GetVariableType().Description.TypeName, variable.Description.Name,
                                variable.Description.Semantic, val.GetVariableType().Description.TypeName,
                                val.Description.Name, getExpectedTypes(valid,val.Description.Name)));
                    }
                    return val;
                }
            }
            return null;
        }

        /// <summary>
        ///     アノテーション名の大文字小文字を無視しして取得する
        /// </summary>
        /// <param name="pass">取得対象のパス</param>
        /// <param name="target">アノテーション名</param>
        /// <param name="typeName">期待する型</param>
        /// <returns>アノテーション</returns>
        public static EffectVariable getAnnotation(EffectPass pass, string target, string typeName)
        {
            string name = target.ToLower();
            string[] valid = name.Split('/');
            for (int i = 0; i < pass.Description.AnnotationCount; i++)
            {
                EffectVariable val = pass.GetAnnotationByIndex(i);
                string typeString = val.Description.Name.ToLower();
                if (typeString == name)
                {
                    if (
                        !valid.Contains(typeString) && !String.IsNullOrWhiteSpace(typeString))
                    {
                        throw new InvalidMMEEffectShaderException(
                            string.Format(
                                "パス「{0}」に適用されたアノテーション「{1} {2}」はアノテーションの型が正しくありません。期待した型は{3}でした。",
                                pass.Description.Name, typeString, val.Description.Name,getExpectedTypes(valid,val.Description.Name)));
                    }
                    return val;
                }
            }
            return null;
        }

        /// <summary>
        ///     アノテーション名の大文字小文字を無視しして取得する
        /// </summary>
        /// <param name="technique">取得対象のテクニック</param>
        /// <param name="target">アノテーション名</param>
        /// <param name="typeName">期待する型</param>
        /// <returns>アノテーション</returns>
        public static EffectVariable getAnnotation(EffectTechnique technique, string target, string typeName)
        {
            string name = target.ToLower();
            string[] valid = name.Split('/');
            for (int i = 0; i < technique.Description.AnnotationCount; i++)
            {
                EffectVariable val = technique.GetAnnotationByIndex(i);
                string typeString = val.Description.Name.ToLower();
                if (typeString == name)
                {
                    if (
                        !valid.Contains(typeString) && !String.IsNullOrWhiteSpace(typeString))
                    {
                        throw new InvalidMMEEffectShaderException(
                            string.Format(
                                "テクニック「{0}」に適用されたアノテーション「{1} {2}」はアノテーションの型が正しくありません。期待した型は{3}でした。"
                                ,technique.Description.Name,typeString,val.Description.Name,getExpectedTypes(valid,val.Description.Name)));
                    }
                    return val;
                }
            }
            return null;
        }

        public static EffectVariable getAnnotation(EffectGroup group, string target, string typeName)
        {
            string name = target.ToLower();
            string[] valid = name.Split('/');
            for (int i = 0; i < group.Description.AnnotationCount; i++)
            {
                EffectVariable val = group.GetAnnotationByIndex(i);
                string typeString = val.Description.Name.ToLower();
                if (typeString == name)
                {
                    if (
                        !valid.Contains(typeString) && !String.IsNullOrWhiteSpace(typeString))
                    {
                        throw new InvalidMMEEffectShaderException(
                            string.Format(
                                "エフェクトグループ「{0}」に適用されたアノテーション「{1} {2}」はアノテーションの型が正しくありません。期待した型は{3}でした。"
                                , group.Description.Name, typeString, val.Description.Name, getExpectedTypes(valid, val.Description.Name)));
                    }
                    return val;
                }
            }
            return null;
        }

        /// <summary>
        ///     指定したテクニックの指定したアノテーションを文字列で取得します
        /// </summary>
        /// <param name="technique">テクニック</param>
        /// <param name="attrName">アノテーション名</param>
        /// <returns>値</returns>
        public static string getAnnotationString(EffectTechnique technique, string attrName)
        {
            EffectVariable annotationVariable = getAnnotation(technique, attrName, "string");
            if (annotationVariable == null) return "";
            return annotationVariable.AsString().GetString();
        }

        /// <summary>
        ///     指定したテクニックの指定したアノテーションをBool値で取得します
        /// </summary>
        /// <param name="technique">テクニック</param>
        /// <param name="attrName">アノテーション名</param>
        /// <returns>値</returns>
        public static ExtendedBoolean getAnnotationBoolean(EffectTechnique technique, string attrName)
        {
            EffectVariable annotationVariable = getAnnotation(technique, attrName, "bool");
            if (annotationVariable == null) return ExtendedBoolean.Ignore;
            int annotation = annotationVariable.AsScalar().GetInt();
            if (annotation == 1)
            {
                return ExtendedBoolean.Enable;
            }
            return ExtendedBoolean.Disable;
        }

        private static string getExpectedTypes(string[] types, string name)
        {
            StringBuilder builder=new StringBuilder();
            foreach (string type in types)
            {
                if(String.IsNullOrWhiteSpace(type))continue;
                if (builder.Length != 0) builder.Append(",");
                builder.Append(string.Format("「{0} {1}」", type, name));
            }
            return builder.ToString();
        }
    }
}