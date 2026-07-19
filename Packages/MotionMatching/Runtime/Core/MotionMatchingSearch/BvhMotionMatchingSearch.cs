using System;
using Unity.Collections;
using Unity.Jobs;

namespace MotionMatching
{
[Serializable]
public class BvhMotionMatchingSearch : MotionMatchingSearch
{
    // Acceleration Structure
    private NativeArray<float> _largeBoundingBoxMin;
    private NativeArray<float> _largeBoundingBoxMax;
    private NativeArray<float> _smallBoundingBoxMin;
    private NativeArray<float> _smallBoundingBoxMax;

    public override void Initialize(
        FeatureSet featureSet, NativeArray<bool> tagMask,
        NativeArray<float> featuresWeights
        )
    {
        FeatureSet = featureSet;
        TagMask = tagMask;
        FeatureWeights = featuresWeights;
        featureSet.GetBVHBuffers(out _largeBoundingBoxMin,
            out _largeBoundingBoxMax,
            out _smallBoundingBoxMin,
            out _smallBoundingBoxMax);
        SearchResult = new NativeArray<int>(2, Allocator.Persistent);
        SearchResult[0] = 0;
        SearchResult[1] = 0;
    }

    public override int FindBestFrame(NativeArray<float> queryFeature, float currentDistance)
    {
        var job = new BVHMotionMatchingSearchBurst
        {
            Valid = FeatureSet.GetValid(),
            TagMask = TagMask,
            Features = FeatureSet.GetFeatures(),
            QueryFeature = queryFeature,
            FeatureWeights = FeatureWeights,
            FeatureSize = FeatureSet.FeatureSize,
            FeatureStaticSize = FeatureSet.FeatureStaticSize,
            CurrentDistance = currentDistance,
            
            LargeBoundingBoxMin = _largeBoundingBoxMin,
            LargeBoundingBoxMax = _largeBoundingBoxMax,
            SmallBoundingBoxMin = _smallBoundingBoxMin,
            SmallBoundingBoxMax = _smallBoundingBoxMax,
            
            BestIndex = SearchResult
        };
        job.Schedule().Complete();

        return SearchResult[0];
    }

    public override void Dispose()
    {
        if (SearchResult.IsCreated) SearchResult.Dispose();
    }
}
}
