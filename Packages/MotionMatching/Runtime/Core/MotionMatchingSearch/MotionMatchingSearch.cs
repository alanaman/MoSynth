using System;
using Unity.Collections;
using UnityEngine;
using static MotionMatching.MotionMatchingData;

namespace MotionMatching
{
[Serializable]
public abstract class MotionMatchingSearch
{
    protected FeatureSet FeatureSet;
    
    [ReadOnly] protected NativeArray<bool> TagMask;
    [ReadOnly] protected NativeArray<float> FeatureWeights; // Size = FeatureSize
    protected NativeArray<int> SearchResult;

    public static MotionMatchingSearch Default => new BvhMotionMatchingSearch();

    public virtual void Initialize(FeatureSet featureSet, NativeArray<bool> tagMask, NativeArray<float> featuresWeights)
    {
        throw new NotImplementedException();
    }

    public abstract int FindBestFrame(NativeArray<float> queryFeature, float currentDistance);
    
    public virtual void Dispose()
    {
    }
}
}