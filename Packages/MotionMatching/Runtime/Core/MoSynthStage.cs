using System;
using UnityEngine;

namespace MotionMatching
{
[Serializable]
public class MoSynthStage
{
    public virtual void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        throw new System.NotImplementedException();
    }

    public virtual Skeleton GetSkeleton(in Skeleton inSkeleton)
    {
        return inSkeleton;
    }

    public virtual void Apply(PoseVector pose, float deltaTime)
    {
        throw new System.NotImplementedException();
    }
}
}