using System.Collections.Generic;
using UnityEngine;

public class DeliveryBoardGUI : MonoBehaviour
{
    [SerializeField]
    private GameObject _panel;
    [SerializeField]
    private DeliveryBoard _deliveryBoard;
    [SerializeField]
    private DeliveryRun _deliveryRun;
    [SerializeField]
    private DeliveryEntryGUI _entryPrefab;
    [ SerializeField]
    private Transform _entryContainer;

    private Dictionary<Delivery, DeliveryEntryGUI> _entries = new();

    private void Awake()
    {
        _deliveryBoard.OnDeliveryGenerated += HandleDeliveryGenerated;
        DeliveryEvents.OnDeliveryAccepted += HandleDeliveryAccepted;
        gameObject.SetActive(false);
    }

    private void OnDestroy()
    {
        _deliveryBoard.OnDeliveryGenerated -= HandleDeliveryGenerated;
        DeliveryEvents.OnDeliveryAccepted -= HandleDeliveryAccepted;
    }

    private void HandleDeliveryGenerated(Delivery delivery)
    {
        var entry = Instantiate(_entryPrefab, _entryContainer);
        entry.Setup(delivery);
        entry.OnAccepted += HandleEntryAccepted;
        _entries[delivery] = entry;
    }

    private void HandleEntryAccepted(Delivery delivery)
    {
        _deliveryRun.AcceptDelivery(delivery);
    }

    private void HandleDeliveryAccepted(Delivery delivery)
    {
        if (!_entries.TryGetValue(delivery, out var entry)) return;
        entry.OnAccepted -= HandleEntryAccepted;
        Destroy(entry.gameObject);
        _entries.Remove(delivery);
        _panel.SetActive(false);
    }
}
