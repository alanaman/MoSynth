using UnityEngine.Playables;

namespace MotionMatching
{
    public class MoSynthPlayableBehaviour : PlayableBehaviour
    {
        public MoSynthStage Stage { get; private set; }
        public PoseVector OutputPose;

        private Playable _thisPlayable;

        public void Initialize(MoSynthStage stage, Skeleton skeleton, Playable playable)
        {
            Stage = stage;
            if (skeleton != null && skeleton.Joints != null)
            {
                OutputPose = new PoseVector(skeleton.Joints.Count);
            }
            _thisPlayable = playable;
        }

        public override void ProcessFrame(Playable playable, FrameData info, object playerData)
        {
            if (Stage == null) return;
            if (!Stage.isEnabled) return;

            // Read input pose from port 0 if available
            if (playable.GetInputCount() > 0)
            {
                var inputPlayable = playable.GetInput(0);
                if (inputPlayable.IsValid())
                {
                    if (inputPlayable.GetPlayableType() == typeof(MoSynthPlayableBehaviour))
                    {
                        var inputBhv = ((ScriptPlayable<MoSynthPlayableBehaviour>)inputPlayable).GetBehaviour();
                        if (inputBhv != null && inputBhv.OutputPose.JointLocalPositions != null)
                        {
                            OutputPose.CopyFrom(inputBhv.OutputPose);
                        }
                    }
                }
            }

            // Let the stage modify the pose
            Stage.Apply(OutputPose, info.deltaTime);
        }
    }
}
