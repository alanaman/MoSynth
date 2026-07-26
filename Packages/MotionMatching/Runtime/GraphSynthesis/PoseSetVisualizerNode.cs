using System;
using Unity.GraphToolkit;
using UnityEngine;

namespace MotionMatching
{
    [Serializable]
    public class PoseSetVisualizerNode : MoSynthStageNode
    {
        public PoseSetVisualizerStage stage = new PoseSetVisualizerStage();

        protected override void OnDefinePorts(IPortDefinitionContext context)
        {
            context.AddInputPort<PoseVector>("Input Pose").Build();
            context.AddOutputPort<PoseVector>("Output Pose").Build();
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
