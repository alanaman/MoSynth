using System;
using Unity.GraphToolkit;
using UnityEngine;

namespace MotionMatching
{
    [Serializable]
    public class MotionMatchingNode : MoSynthStageNode
    {
        public MotionMatchingStage stage = new MotionMatchingStage();

        protected override void OnDefinePorts(IPortDefinitionContext context)
        {
            context.AddInputPort<PoseVector>("Previous Pose").Build();
            context.AddOutputPort<PoseVector>("Matched Pose").Build();
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
