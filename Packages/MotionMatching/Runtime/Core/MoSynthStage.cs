using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace MotionMatching
{
[Serializable]
public abstract class MoSynthStage
{
    public bool isEnabled = true;
    
    /// <summary>
    /// Used by the <see cref="MotionSynthesisComponent"/> to get the skeleton.
    /// Called before <see cref="Init"/>
    /// </summary>
    /// <param name="inSkeleton">
    /// The output skeleton from the previous <see cref="MoSynthStage"/>.
    /// Will be null if this is the first stage.
    /// </param>
    /// <returns>The modified skeleton.</returns>
    public virtual Skeleton GetSkeleton(in Skeleton inSkeleton)
    {
        return inSkeleton;
    }
    
    public virtual void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        throw new System.NotImplementedException();
    }

    public virtual void OnValidate()
    {
    }

    public abstract void Apply(PoseVector pose, float deltaTime);
}
}