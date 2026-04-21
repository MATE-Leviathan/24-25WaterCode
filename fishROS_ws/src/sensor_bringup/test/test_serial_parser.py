from sensor_bringup.serial_parser import parse_serial_json_line


def test_parse_serial_json_line_extracts_numeric_and_string_fields():
    parsed = parse_serial_json_line(
        '{"temp_c":21.4,"ph":7.1,"status":"ok","wet":true}'
    )

    assert parsed.parse_ok
    assert parsed.error == ''
    assert parsed.numeric_keys == ['temp_c', 'ph']
    assert parsed.numeric_values == [21.4, 7.1]
    assert parsed.string_keys == ['status', 'wet']
    assert parsed.string_values == ['ok', 'true']


def test_parse_serial_json_line_preserves_bad_input():
    parsed = parse_serial_json_line('not-json')

    assert not parsed.parse_ok
    assert parsed.raw_line == 'not-json'
    assert 'json decode error' in parsed.error
