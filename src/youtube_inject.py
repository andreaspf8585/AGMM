#!/usr/bin/env python3
"""
youtubeload_inject.py
---------------------
For every *.cpp file in the current folder:

  1. Prepends `#include "YouTubeLoad.hpp"` at the top of the file
     (skipped if the include is already present).

  2. Inserts `agmm::addYouTubeLink(menu); // adds "Visit my youtube channel!"`
     as the first statement inside `appendContextMenu(Menu*)`, using the
     method's actual parameter name.
     (Skipped if a call to addYouTubeLink is already present;
      files without an appendContextMenu definition are left untouched.)

Both steps are idempotent — running the script twice is safe.

Usage:
    python3 youtubeload_inject.py             # modify files in place
    python3 youtubeload_inject.py --dry-run   # preview only
"""

import re
import sys
from pathlib import Path
from typing import Tuple

INCLUDE_LINE = '#include "YouTubeLoad.hpp"'
CALL_PREFIX  = 'agmm::addYouTubeLink('   # broad marker for idempotency check


# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------
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
# Step 2: insert the call at the top of appendContextMenu
# ---------------------------------------------------------------------------
def inject_call(text: str) -> Tuple[str, str]:
    """Return (new_text, message). If new_text == text, nothing was changed."""
    if CALL_PREFIX in text:
        return text, 'already has addYouTubeLink call'

    m = re.search(r'\bappendContextMenu\s*\(\s*Menu\s*\*\s*(\w+)\s*\)', text)
    if not m:
        return text, 'no appendContextMenu definition found'

    menu_var  = m.group(1)
    call_line = f'agmm::addYouTubeLink({menu_var}); // adds "Visit my youtube channel!"'

    # Skip forward declarations (`... override;`)
    after = text[m.end():m.end() + 200]
    m2 = re.match(r'\s*(?:override\s*|final\s*)*', after)
    rest = m.end() + (m2.end() if m2 else 0)
    if rest < len(text) and text[rest] == ';':
        return text, 'appendContextMenu is a forward declaration'

    brace = text.find('{', m.end())
    if brace == -1:
        return text, 'appendContextMenu found but no opening brace'

    base_indent  = line_indent_of(text, brace)
    inner_indent = base_indent + '    '

    insert_pos   = brace + 1
    content_pos  = insert_pos
    while content_pos < len(text) and text[content_pos] in ' \t\r\n':
        content_pos += 1

    insertion = '\n' + inner_indent + call_line + '\n' + inner_indent
    return (
        text[:insert_pos] + insertion + text[content_pos:],
        'injected into appendContextMenu',
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
            elif msg not in ('already has addYouTubeLink call',
                             'no appendContextMenu definition found'):
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