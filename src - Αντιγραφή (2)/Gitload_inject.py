#!/usr/bin/env python3
"""
gitload_inject.py
-----------------
For every *.cpp file in the current folder:

  1. Prepends `#include "GitLoad.hpp"` at the top of the file
     (skipped if the include is already present).

  2. Ensures `agmm::addGitLink(menu);` is called inside
     `appendContextMenu(Menu*)`:
        - if the method already exists, the call is inserted as the
          first statement of its body (using the actual parameter name);
        - if it does not exist, a complete
          `void appendContextMenu(Menu *menu) override { ... }` is
          appended to the `struct ... : ModuleWidget` definition.
     (Skipped if a call to addGitLink is already present.)

Both steps are idempotent — running the script twice is safe.

Usage:
    python3 gitload_inject.py             # modify files in place
    python3 gitload_inject.py --dry-run   # preview only
"""

import re
import sys
from pathlib import Path
from typing import Tuple

INCLUDE_LINE = '#include "GitLoad.hpp"'
CALL_PREFIX  = 'agmm::addGitLink('   # broad marker for idempotency check


# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------
def find_matching_brace(text: str, open_pos: int) -> int:
    """Return index of the '}' matching text[open_pos] == '{', or -1."""
    if open_pos < 0 or open_pos >= len(text) or text[open_pos] != '{':
        return -1
    depth = 0
    i = open_pos
    n = len(text)
    while i < n:
        c = text[i]
        # line comment
        if c == '/' and i + 1 < n and text[i + 1] == '/':
            nl = text.find('\n', i + 2)
            i = n if nl == -1 else nl
            continue
        # block comment
        if c == '/' and i + 1 < n and text[i + 1] == '*':
            end = text.find('*/', i + 2)
            i = n if end == -1 else end + 2
            continue
        # string literal
        if c == '"':
            i += 1
            while i < n:
                if text[i] == '\\':
                    i += 2
                    continue
                if text[i] == '"':
                    i += 1
                    break
                i += 1
            continue
        # char literal
        if c == "'":
            i += 1
            while i < n:
                if text[i] == '\\':
                    i += 2
                    continue
                if text[i] == "'":
                    i += 1
                    break
                i += 1
            continue
        if c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return -1


def line_indent_of(text: str, pos: int) -> str:
    """Return the leading whitespace of the line that contains `pos`."""
    start = text.rfind('\n', 0, pos) + 1
    j = start
    while j < pos and text[j] in ' \t':
        j += 1
    return text[start:j]


# ---------------------------------------------------------------------------
# Step 1: prepend the include
# ---------------------------------------------------------------------------
def inject_include(text: str) -> str:
    if INCLUDE_LINE in text:
        return text
    bom = ''
    if text.startswith('\ufeff'):
        bom = '\ufeff'
        text = text[1:]
    return bom + INCLUDE_LINE + '\n' + text


# ---------------------------------------------------------------------------
# Step 2: ensure agmm::addGitLink(...) runs inside appendContextMenu
# ---------------------------------------------------------------------------
def inject_call(text: str) -> Tuple[str, str]:
    """Return (new_text, message). If new_text == text, nothing was changed."""
    if CALL_PREFIX in text:
        return text, 'already has addGitLink call'

    # --- Case 1: appendContextMenu already defined ------------------------
    m = re.search(r'\bappendContextMenu\s*\(\s*Menu\s*\*\s*(\w+)\s*\)', text)
    if m:
        menu_var = m.group(1)
        call_line = f'agmm::addGitLink({menu_var}); // adds "Visit my git repo!"'

        # Skip forward declarations of the form `... override;`
        after = text[m.end():m.end() + 200]
        m2 = re.match(r'\s*(?:override\s*|final\s*)*', after)
        rest = m.end() + (m2.end() if m2 else 0)
        if rest < len(text) and text[rest] == ';':
            return text, 'appendContextMenu is a forward declaration'

        brace = text.find('{', m.end())
        if brace == -1:
            return text, 'appendContextMenu found but no opening brace'

        base_indent = line_indent_of(text, brace)
        inner_indent = base_indent + '    '

        insert_pos = brace + 1
        content_pos = insert_pos
        while content_pos < len(text) and text[content_pos] in ' \t\r\n':
            content_pos += 1

        insertion = '\n' + inner_indent + call_line + '\n' + inner_indent
        return (
            text[:insert_pos] + insertion + text[content_pos:],
            'injected into existing appendContextMenu',
        )

    # --- Case 2: add a whole new method to the ModuleWidget struct --------
    sm = re.search(
        r'struct\s+\w+\s*:\s*(?:public\s+)?(?:\w+::)?ModuleWidget\b', text
    )
    if not sm:
        return text, 'no ModuleWidget struct found (call not added)'

    open_brace = text.find('{', sm.end())
    if open_brace == -1:
        return text, 'ModuleWidget struct has no body'

    close_brace = find_matching_brace(text, open_brace)
    if close_brace == -1:
        return text, 'could not locate ModuleWidget closing brace'

    struct_indent = line_indent_of(text, sm.start())
    member_indent = struct_indent + '    '
    body_indent   = member_indent + '    '

    method_body = (
        f'{member_indent}void appendContextMenu(Menu *menu) override\n'
        f'{member_indent}{{\n'
        f'{body_indent}agmm::addGitLink(menu); // adds "Visit my git repo!"\n'
        f'{body_indent}// ... your other menu items ...\n'
        f'{member_indent}}}\n'
    )

    # Separator newlines (avoid gluing onto the previous line / last member)
    prefix = ''
    if close_brace > 0 and text[close_brace - 1] != '\n':
        prefix = '\n'
    if close_brace > 1 and text[close_brace - 2] != '\n':
        prefix = '\n' + prefix

    return (
        text[:close_brace] + prefix + method_body + text[close_brace:],
        'created new appendContextMenu in ModuleWidget',
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> int:
    dry_run = '--dry-run' in sys.argv
    folder = Path.cwd()
    files = sorted(folder.glob('*.cpp'))
    if not files:
        print(f'No .cpp files found in {folder}')
        return 0

    modified = 0
    skipped  = 0

    for f in files:
        try:
            text = f.read_text(encoding='utf-8')
        except UnicodeDecodeError:
            text = f.read_text(encoding='latin-1')

        original = text
        actions  = []
        warnings = []

        # Step 1: include
        if INCLUDE_LINE not in text:
            text = inject_include(text)
            actions.append(f'added {INCLUDE_LINE}')

        # Step 2: call inside appendContextMenu
        if CALL_PREFIX not in text:
            before_call = text
            text, msg = inject_call(text)
            if text != before_call:
                actions.append(msg)
            elif msg != 'already has addGitLink call':
                warnings.append(msg)

        for w in warnings:
            print(f'[warn] {f.name}: {w}')

        if text == original:
            print(f'[skip] {f.name}')
            skipped += 1
            continue

        if dry_run:
            print(f'[would modify] {f.name}')
            for a in actions:
                print(f'    - {a}')
            modified += 1
            continue

        f.write_text(text, encoding='utf-8')
        print(f'[modify] {f.name}')
        for a in actions:
            print(f'    - {a}')
        modified += 1

    print(f'\nDone. {modified} modified, {skipped} skipped.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())