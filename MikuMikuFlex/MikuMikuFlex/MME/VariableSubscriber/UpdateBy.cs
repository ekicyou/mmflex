namespace MMF.MME.VariableSubscriber
{
    /// <summary>
    ///     変数が何毎に更新されるか
    /// </summary>
    public enum UpdateBy
    {
        /// <summary>
        ///     この変数はマテリアルごとに更新される
        /// </summary>
        Material,

        /// <summary>
        ///     この変数はモデルごとに更新される
        /// </summary>
        Model
    }
}