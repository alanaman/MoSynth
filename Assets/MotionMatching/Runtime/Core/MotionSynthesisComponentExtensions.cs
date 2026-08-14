using AnimationTools;

namespace MotionMatching
{
    public static class MotionSynthesisComponentExtensions
    {
        /// <summary>
        /// The <see cref="MotionMatchingData"/> of the first <see cref="MotionMatchingStage"/> in
        /// <see cref="MotionSynthesisComponent.stages"/>, or null when no such stage exists (e.g. a
        /// motion-field-only stack).
        /// </summary>
        public static MotionMatchingData GetMmData(this MotionSynthesisComponent msc)
        {
            foreach (var stage in msc.stages)
            {
                if (stage is MotionMatchingStage mm) return mm.mmData;
            }

            return null;
        }
    }
}
