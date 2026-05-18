from rhw_udp_mqtt_bridge.album_payload import build_album_payload


def test_plain_album_payload_matches_platform_sample_shape():
    payload = build_album_payload(
        trace_id='177682537407120985',
        partner_id='1000009',
        version='1.0',
        device_id='DOG001',
        image_base64='/9j/test',
        task_id='2042538076389556225',
        point_name='大门口',
        point_id='10001',
        encryption_enabled=False,
        encrypt_data=lambda data_text: data_text,
        signature_enabled=False,
        fixed_signature='4ce70638eae5991dd10064a534d00a21',
        signature_secret='',
        include_device_id=False,
    )

    assert payload == {
        'traceId': '177682537407120985',
        'partnerId': '1000009',
        'version': '1.0',
        'data': {
            'taskId': '2042538076389556225',
            'pointName': '大门口',
            'pointId': '10001',
            'base64': '/9j/test',
        },
        'signature': '4ce70638eae5991dd10064a534d00a21',
    }
