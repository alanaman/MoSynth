using UnityEngine;

namespace MotionMatching
{
public class MoSynthStage
{
    public virtual void Init(MotionSynthesizer motionSynthesizer)
    {
        throw new System.NotImplementedException();
    }

    public virtual PoseVector Apply(PoseVector inPose, float deltaTime)
    {
        throw new System.NotImplementedException();
    }
}
}