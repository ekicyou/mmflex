namespace MMF.Model.PMX
{
    public interface IPMXSubset : ISubset
    {
        int StartIndex { get; }
        int VertexCount { get;}
    }
}