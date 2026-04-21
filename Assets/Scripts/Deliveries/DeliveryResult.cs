public class DeliveryResult
{
    public bool Success { get; }
    public int BaseReward { get; }
    public int DamagePenalty { get; }
    public int FinalReward { get; }

    public DeliveryResult(bool success, int baseReward, int damagePenalty, int finalReward)
    {
        Success = success;
        BaseReward = baseReward;
        DamagePenalty = damagePenalty;
        FinalReward = finalReward;
    }
}