using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class DeliveryEntryGUI : MonoBehaviour
{
    [SerializeField]
    private TextMeshProUGUI _startText;
    [SerializeField]
    private TextMeshProUGUI _endText;
    [SerializeField]
    private TextMeshProUGUI _rewardText;
    [SerializeField]
    private Button _acceptButton;

    private Delivery _delivery;

    public Action<Delivery> OnAccepted;

    public void Setup(Delivery delivery)
    {
        _delivery = delivery;
        _startText.text = $"From: {_delivery.StartPoint.Location}";
        _endText.text = $"To: {_delivery.EndPoint.Location}";
        _rewardText.text = $"Reward: {_delivery.Reward} kr.";
        _acceptButton.onClick.AddListener(HandleAcceptClicked);
    }

    private void HandleAcceptClicked()
    {
        OnAccepted?.Invoke(_delivery);
    }

    private void OnDestroy()
    {
        _acceptButton.onClick.RemoveListener(HandleAcceptClicked);
    }
}
