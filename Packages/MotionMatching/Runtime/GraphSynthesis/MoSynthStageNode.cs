using System;
using Unity.GraphToolkit;
using UnityEngine;

namespace MotionMatching
{
    [Serializable]
    public abstract class MoSynthStageNode : Node
    {
        public virtual Skeleton GetSkeleton(Skeleton inSkeleton)
        {
            return inSkeleton;
        }

        public abstract void Init(GraphSynthesisComponent motionSynthesisComponent);

        public abstract void Apply(PoseVector pose, float deltaTime);
    }
}
