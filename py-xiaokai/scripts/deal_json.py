import re
def _remove_json_from_result(s: str):
    """
    Find the first occurrence of backslashes followed by 'json' (e.g. \\json or \json),
    then the following brace block (supports one or more opening braces like {{ or {),
    and returns (text_before, json_block_including_braces).
    If no json marker found, returns (s, None).
    The brace-matching ignores braces inside quotes (single/double) and handles escapes.
    """
    m = re.search(r'\\+json', s)
    if not m:
        return s, None

    # find index of first '{' after the 'json' marker
    i = m.end()
    # skip whitespace between json and brace if any
    while i < len(s) and s[i].isspace():
        i += 1
    if i >= len(s) or s[i] != '{':
        # no opening brace found
        return s[:m.start()], None

    # count how many consecutive '{' there are (e.g. '{{' -> 2)
    start = i
    while i < len(s) and s[i] == '{':
        i += 1
    # we'll parse from start to find matching number of closing braces
    required_open = i - start

    # parse with awareness of quotes and escapes
    open_count = 0
    in_string = False
    string_char = ''
    idx = start
    while idx < len(s):
        ch = s[idx]
        # determine if this char is escaped by counting preceding backslashes
        # an odd number of preceding backslashes means it is escaped
        bs_count = 0
        j = idx - 1
        while j >= 0 and s[j] == '\\':
            bs_count += 1
            j -= 1
        escaped = (bs_count % 2 == 1)

        if not in_string:
            if ch == '"' or ch == "'":
                in_string = True
                string_char = ch
            elif ch == '{':
                open_count += 1
            elif ch == '}':
                open_count -= 1
                # if we've closed all braces (and we've seen at least required_open opens), stop
                if open_count == 0:
                    # include this closing brace
                    json_text = s[start:idx+1]
                    return s[:m.start()], json_text
        else:
            # inside string
            if ch == string_char and not escaped:
                in_string = False
                string_char = ''
        idx += 1

    # if we exit loop without closing braces, return what we can
    return s[:m.start()], s[start:]

text = "好的，切换到腹腔镜模式。 \json{{'command_type': 'mode_switch', 'action': 'set', 'parameters': {'mode': 'endoscope'}}}"
print(_remove_json_from_result(text))

text = "\\"
if text.startswith('\\'):
    print(True)