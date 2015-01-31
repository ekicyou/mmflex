using System;

namespace MMF.Model.Assimp
{
    [Flags]
    public enum AssimpFileFilter
    {
        AllFile=0x0F
        ,CommonModelFile=0x01
        ,CommonGameEngineFile=0x02,CommonGameFile=0x04,OtherFile=0x08
    }
}