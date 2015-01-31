namespace MMF.MME.VariableSubscriber.ControlInfoSubscriber
{
    /// <summary>
    /// コントロールオブジェクトのアノテーションをまとめたもの
    /// </summary>
    class ControlObjectAnnotation
    {
        public ControlObjectAnnotation(TargetObject target, bool isString)
        {
            Target = target;
            IsString = isString;
        }

        public TargetObject Target { get; private set; }

        public bool IsString { get; private set; }
    }
}
