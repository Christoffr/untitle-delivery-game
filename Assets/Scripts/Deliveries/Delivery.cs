using UnityEngine;

[System.Serializable]
public class Delivery
{
     [SerializeField] private DeliveryPoint _start;
    [SerializeField] private DeliveryPoint _end;

    public DeliveryPoint StartPoint => _start;
    public DeliveryPoint EndPoint => _end;
    public int Reward { get; private set; }

    public Delivery(DeliveryPoint start, DeliveryPoint end)
    {
        _start = start;
        _end = end;
        float distance = Vector3.Distance(start.transform.position, end.transform.position);
        Reward = CalculateReward(distance);
    }

    private int CalculateReward(float distance)
    {
        return Mathf.RoundToInt((distance / 1000) * 10);
    }
}