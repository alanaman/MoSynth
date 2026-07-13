using Unity.Collections;
using UnityEngine;
using static MotionMatching.MotionMatchingData;

namespace MotionMatching
{
public abstract class MotionMatchingSearch
{
    public static MotionMatchingSearch Default => new BvhMotionMatchingSearch();

    public virtual void Initialize(MotionMatchingController controller)
    {
    }

    public virtual void OnEnabled()
    {
    }

    public virtual void OnDisabled()
    {
    }

    public virtual bool ShouldSearch(MotionMatchingController controller)
    {
        return controller.SearchTimeLeft <= 0;
    }

    public abstract int FindBestFrame(MotionMatchingController controller, float currentDistance);

    public virtual void OnSearchCompleted(MotionMatchingController controller)
    {
    }

    public virtual float OnUpdateEnvironmentFeatureWeight(MotionMatchingController controller,
        TrajectoryFeature environmentFeature, float defaultWeight)
    {
        return defaultWeight;
    }

    public virtual void Dispose()
    {
    }
}
}