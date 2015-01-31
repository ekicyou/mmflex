using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Security.Policy;
using System.Text;
using System.Threading.Tasks;

namespace MMF.Utility
{
    /// <summary>
    /// 階層構造順に要素を取得できるようなコレクション
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public class HierarchicalOrderCollection<T>:List<T>
    {
        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="baseList">ソートしたい要素の配列。baseListはインデックス順に並んでいることを前提とする</param>
        /// <param name="solver">親要素の取得などに使われるインターフェース</param>
        public HierarchicalOrderCollection(T[] baseList,HierarchicalOrderSolver<T> solver)
        {
             Queue<int> cachedQueue=new Queue<int>();//baseListから親の順に格納するキュー
            HashSet<int> cachedSet=new HashSet<int>();//格納された要素をチェックするためのハッシュセット
            cachedSet.Add(-1);
            while (cachedQueue.Count!=baseList.Length)
            {
                foreach (var element in baseList)
                {
                    int index = solver.getIndex(element);
                    int parent = solver.getParentIndex(element);
                    if (cachedSet.Contains(parent)&&!cachedSet.Contains(index))
                    {//もし、親要素がすでに含まれていたとしたら
                        
                        cachedQueue.Enqueue(index);
                        cachedSet.Add(index);
                    }
                }
            }
            while (cachedQueue.Count!=0)
            {
                Add(baseList[cachedQueue.Dequeue()]);
            }
        }

    }

    public interface HierarchicalOrderSolver<T>
    {
        /// <summary>
        /// 親のインデックスを返す
        /// </summary>
        /// <param name="child">調べたい子</param>
        /// <returns>-1の場合は親がいないとする</returns>
        int getParentIndex(T child);

        /// <summary>
        /// 指定したボーンのインデックスを返す
        /// </summary>
        /// <param name="target"></param>
        /// <returns></returns>
        int getIndex(T target);
    }
}
