using System;
using Unity.GraphToolkit;
using UnityEngine;

namespace MotionMatching
{
    [Serializable]
    public class RetargetingNode : MoSynthStageNode
    {
        public RetargetingStage stage = new RetargetingStage();

        protected override void OnDefinePorts(IPortDefinitionContext context)
        {
            context.AddInputPort<PoseVector>("Input Pose").Build();
            context.AddOutputPort<PoseVector>("Retargeted Pose").Build();
        }

        public override Skeleton GetSkeleton(Skeleton inSkeleton)
        {
            return stage.GetSkeleton(inSkeleton);
        }

        public override void Init(GraphSynthesisComponent motionSynthesisComponent)
        {
            stage.Init(motionSynthesisComponent);
        }

        public override void Apply(PoseVector pose, float deltaTime)
        {
            stage.Apply(pose, deltaTime);
        }
    }
}
