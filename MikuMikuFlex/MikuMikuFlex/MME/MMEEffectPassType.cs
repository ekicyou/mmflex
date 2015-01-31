namespace MMF.MME
{
    /// <summary>
    ///     MMEのパスのタイプ
    ///     詳しくはMME仕様158行目近辺参照
    /// </summary>
    public enum MMEEffectPassType
    {
        /// <summary>
        ///     object_ss
        /// </summary>
        Object_SelfShadow,

        /// <summary>
        ///     object
        /// </summary>
        Object,

        /// <summary>
        ///     zplot
        /// </summary>
        ZPlot,

        /// <summary>
        ///     shadow
        /// </summary>
        Shadow,

        /// <summary>
        ///     edge(PMDのみ)
        /// </summary>
        Edge
    }
}