using UnityEngine;

[System.Serializable]
public class Delivery
{
    [SerializeField] private DeliveryPoint start;
    [SerializeField] private DeliveryPoint end;
    [SerializeField] private int reward;

    public DeliveryPoint Start => start;
    public DeliveryPoint End => end;
    public int Reward => reward;

    public Delivery(DeliveryPoint start, DeliveryPoint end, int reward)
    {
        this.start = start;
        this.end = end;
        this.reward = reward;
    }
}
