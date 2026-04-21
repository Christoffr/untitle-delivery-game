using UnityEngine;

public class MoneyManager : MonoBehaviour
{
    [SerializeField]
    private int _money = 0;
    [SerializeField]
    private DeliveryScore score;

    private void Start()
    {
        score.OnDeliveryResult += UpdateMoney;
    }

    private void UpdateMoney(DeliveryResult result)
    {
        _money += result.FinalReward;
        Debug.Log($"Current Money: {_money}");
    }
}
