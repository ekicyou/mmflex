using SlimDX;
using SlimDX.Direct3D11;

namespace MMF.MME.VariableSubscriber.MatrixSubscriber
{
    /// <summary>
    ///     行列の登録クラスのベース
    /// </summary>
    public abstract class MatrixSubscriberBase : SubscriberBase
    {
        protected ObjectAnnotationType TargetObject;

        protected MatrixSubscriberBase(ObjectAnnotationType Object)
        {
            TargetObject = Object;
        }

        protected MatrixSubscriberBase()
        {
        }

        public override VariableType[] Types
        {
            get { return new[] {VariableType.Float4x4}; }
        }

        /// <summary>
        ///     行列としてエフェクトに登録する
        /// </summary>
        /// <param name="matrix">登録する行列</param>
        /// <param name="effect">エフェクト</param>
        /// <param name="index">変数のインデックス</param>
        protected void SetAsMatrix(Matrix matrix, EffectVariable variable)
        {
            variable.AsMatrix().SetMatrix(matrix);
        }

        /// <summary>
        ///     行列の場合は、それぞれCameraかLightか調べる
        /// </summary>
        /// <param name="variable"></param>
        /// <param name="context"></param>
        /// <param name="effectManager"></param>
        /// <param name="semanticIndex"></param>
        /// <param name="effect">検査対象のエフェクト</param>
        /// <param name="index">検査対象の変数のindex</param>
        /// <returns></returns>
        public override SubscriberBase GetSubscriberInstance(EffectVariable variable, RenderContext context, MMEEffectManager effectManager, int semanticIndex)
        {
            string obj;
            EffectVariable annotation = EffectParseHelper.getAnnotation(variable, "Object", "string");
            obj = annotation == null ? "" : annotation.AsString().GetString(); //アノテーションが存在しない時は""とする
            if (string.IsNullOrWhiteSpace(obj)) return GetSubscriberInstance(ObjectAnnotationType.Camera);
            switch (obj.ToLower())
            {
                case "camera":
                    return GetSubscriberInstance(ObjectAnnotationType.Camera);
                case "light":
                    return GetSubscriberInstance(ObjectAnnotationType.Light);
                case "":
                    throw new InvalidMMEEffectShaderException(
                        string.Format(
                            "変数「{0} {1}:{2}」には、アノテーション「string Object=\"Camera\"」または、「string Object=\"Light\"」が必須ですが指定されませんでした。",
                            variable.GetVariableType().Description.TypeName.ToLower(), variable.Description.Name,
                            variable.Description.Semantic));
                default:
                    throw new InvalidMMEEffectShaderException(
                        string.Format(
                            "変数「{0} {1}:{2}」には、アノテーション「string Object=\"Camera\"」または、「string Object=\"Light\"」が必須ですが指定されたのは「string Object=\"{3}\"」でした。(スペルミス?)",
                            variable.GetVariableType().Description.TypeName.ToLower(), variable.Description.Name,
                            variable.Description.Semantic, obj));
            }
        }

        protected abstract SubscriberBase GetSubscriberInstance(ObjectAnnotationType Object);
    }
}