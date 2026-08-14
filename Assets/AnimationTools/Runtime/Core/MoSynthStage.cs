using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace AnimationTools
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
    public virtual SkeletonAsset GetSkeleton(SkeletonAsset inSkeleton)
    {
        return inSkeleton;
    }
    
    public abstract void Init(MotionSynthesisComponent motionSynthesisComponent);

    public virtual void OnValidate()
    {
    }

    /// <summary>
    /// Transforms the pipeline pose in place. <see cref="PoseBuffer"/> is a view struct, so
    /// the by-value parameter aliases the caller's pose data.
    /// </summary>
    /// <param name="pose">The pipeline pose; its layout is the owning component's PoseLayout.</param>
    /// <param name="deltaTime"></param>
    /// <returns>Return false to tell MSC to stop at this stage.</returns>
    public abstract bool Apply(PoseBuffer pose, float deltaTime);

    public virtual void OnDestroy()
    {
    }
}
}