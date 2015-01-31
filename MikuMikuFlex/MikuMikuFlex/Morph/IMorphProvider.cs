using System.Collections.Generic;
using MMF.Motion;

namespace MMF.Morph
{
    public interface IMorphProvider
    {
        void ApplyMorphProgress(float frameNumber, IEnumerable<MorphMotion> morphMotions);

        bool ApplyMorphProgress(float progress, string morphName);

        void UpdateFrame();

    }
}