using Unity.Collections;
using Unity.Jobs;

namespace MotionMatching
{
public class BvhMotionMatchingSearch : MotionMatchingSearch
{
    private NativeArray<int> _searchResult;
    private bool _isDisposed = false;

    // Acceleration Structure
    private NativeArray<float> _largeBoundingBoxMin;
    private NativeArray<float> _largeBoundingBoxMax;
    private NativeArray<float> _smallBoundingBoxMin;
    private NativeArray<float> _smallBoundingBoxMax;

    public override void Initialize(MotionMatchingController controller)
    {
        controller.FeatureSet.GetBVHBuffers(out _largeBoundingBoxMin,
            out _largeBoundingBoxMax,
            out _smallBoundingBoxMin,
            out _smallBoundingBoxMax);

        _searchResult = new NativeArray<int>(2, Allocator.Persistent);
        _searchResult[0] = 0;
        _searchResult[1] = 0;

        _isDisposed = false;
    }

    public override int FindBestFrame(MotionMatchingController controller, float currentDistance)
    {
        if (_isDisposed) return controller.CurrentFrame;

        var job = new BVHMotionMatchingSearchBurst
        {
            Valid = controller.FeatureSet.GetValid(),
            TagMask = controller.TagMask,
            Features = controller.FeatureSet.GetFeatures(),
            QueryFeature = controller.QueryFeature,
            FeatureWeights = controller.FeaturesWeightsNativeArray,
            FeatureSize = controller.FeatureSet.FeatureSize,
            FeatureStaticSize = controller.FeatureSet.FeatureStaticSize,
            PoseOffset = controller.FeatureSet.PoseOffset,
            CurrentDistance = currentDistance,
            LargeBoundingBoxMin = _largeBoundingBoxMin,
            LargeBoundingBoxMax = _largeBoundingBoxMax,
            SmallBoundingBoxMin = _smallBoundingBoxMin,
            SmallBoundingBoxMax = _smallBoundingBoxMax,
            BestIndex = _searchResult
        };
        job.Schedule().Complete();

        return _searchResult[0];
    }

    public override void OnSearchCompleted(MotionMatchingController controller)
    {
    }

    public override void OnEnabled()
    {
    }

    public override void OnDisabled()
    {
    }

    public override void Dispose()
    {
        if (_searchResult.IsCreated) _searchResult.Dispose();
        _isDisposed = true;
    }
}
}
