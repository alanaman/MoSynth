using UnityEngine;
using XCharts.Runtime;

public class JointLivePlotter : MonoBehaviour
{
    [Header("Target")]
    public Transform targetJoint;

    [Header("Chart Settings")]
    public LineChart chart;
    [Tooltip("How many seconds of data to show on the graph")]
    public float timeWindow = 5f;
    [Tooltip("How often to sample the data (seconds)")]
    public float updateInterval = 0.05f;

    private float timer = 0f;
    private Vector3 lastPosition;
    private Vector3 lastVelocity;

    void Start()
    {
        if (chart == null) chart = GetComponent<LineChart>();
        
        lastPosition = targetJoint.position;
        chart.ClearData();
        
        UpdateTimeWindow();
    }

    void Update()
    {
        if (targetJoint == null) return;

        timer += Time.deltaTime;
        
        // Only sample data at the specified interval for performance and readability
        if (timer >= updateInterval)
        {
            PlotData();
            timer = 0f;
        }
    }

    private void PlotData()
    {
        // 1. Calculate Kinematics (Finite Differences)
        Vector3 currentPosition = targetJoint.position;
        Vector3 currentVelocity = (currentPosition - lastPosition) / updateInterval;
        Vector3 currentAcceleration = (currentVelocity - lastVelocity) / updateInterval;

        // Use magnitude for overall movement. Change to .x, .y, or .z for specific axes.
        float pos = currentPosition.magnitude;
        float vel = currentVelocity.magnitude;
        float acc = currentAcceleration.magnitude;

        // 2. Add Data to Chart
        string timeStr = Time.time.ToString("F1");
        
        chart.AddXAxisData(timeStr);
        chart.AddData(0, pos); // Serie 0: Position
        chart.AddData(1, vel); // Serie 1: Velocity
        chart.AddData(2, acc); // Serie 2: Acceleration

        // 3. Cache states for next frame calculation
        lastPosition = currentPosition;
        lastVelocity = currentVelocity;
    }

    // Call this if you change the timeWindow variable at runtime
    public void UpdateTimeWindow()
    {
        int maxDataPoints = Mathf.CeilToInt(timeWindow / updateInterval);

        // Limit X-Axis cache to create the sliding time window effect
        var xAxis = chart.GetChartComponent<XAxis>();
        if (xAxis != null) xAxis.maxCache = maxDataPoints;

        // Limit each serie's cache
        foreach (var serie in chart.series)
        {
            serie.maxCache = maxDataPoints;
        }
    }
}
