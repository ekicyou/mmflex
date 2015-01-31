using System.Collections.Generic;

namespace MMF.MME.Script
{
    internal class FunctionDictionary:Dictionary<string,FunctionBase>
    {

        public void Add(FunctionBase item)
        {
            base.Add(item.FunctionName,item);
        }
    }
}
