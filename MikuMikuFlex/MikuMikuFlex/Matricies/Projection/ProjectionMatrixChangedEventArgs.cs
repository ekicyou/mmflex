using System;

namespace MMF.Matricies.Projection
{
    public class ProjectionMatrixChangedEventArgs:EventArgs
    {
        /// <summary>
        /// 変更された変数
        /// </summary>
        public ProjectionMatrixChangedVariableType ChangedType { get; private set; }

        public ProjectionMatrixChangedEventArgs(ProjectionMatrixChangedVariableType type)
        {
            this.ChangedType = type;
        }
    }
}
