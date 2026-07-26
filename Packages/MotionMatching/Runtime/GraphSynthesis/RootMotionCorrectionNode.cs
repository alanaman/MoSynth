using System;
using Unity.GraphToolkit;
using UnityEngine;

namespace MotionMatching
{
    [Serializable]
    public class RootMotionCorrectionNode : MoSynthStageNode
    {
        public RootMotionCorrectionStage stage = new RootMotionCorrectionStage();

        protected override void OnDefinePorts(IPortDefinitionContext context)
        {
            context.AddInputPort<PoseVector>("Input Pose").Build();
            context.AddOutputPort<PoseVector>("Corrected Pose").Build();
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
