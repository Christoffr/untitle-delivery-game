public class DeliveryResult
{
    public bool Success { get; }
    public int BaseReward { get; }
    public int DamagePenalty { get; }
    public float Modifier { get; }
    public int FinalReward { get; }

    public DeliveryResult(bool success, int baseReward, int damagePenalty, float modifier, int finalReward)
    {
        Success = success;
        BaseReward = baseReward;
        DamagePenalty = damagePenalty;
        Modifier = modifier;
        FinalReward = finalReward;
    }
}