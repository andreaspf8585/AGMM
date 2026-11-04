
import requests
import re
import sys

from pathlib import Path
from datetime import datetime

# ============================================================
# AGMM AUTOMATIC DOCUMENTATION SYSTEM
# Ollama + Qwen3-Coder 30B
# VCV Rack 2
# ============================================================

MODEL = "qwen3-coder:30b"

PROJECT_DIR = Path(__file__).resolve().parent
SOURCE_DIR = PROJECT_DIR / "src"
DOCS_DIR = PROJECT_DIR / "docs"

OLLAMA_URL = "http://localhost:11434/api/generate"

EXTENSIONS = {".cpp", ".hpp"}

TEMPERATURE = 0.2
NUM_CTX = 16384

# ============================================================
# INITIALIZATION
# ============================================================

if not SOURCE_DIR.is_dir():
    print("ERROR: src directory not found!")
    print(SOURCE_DIR)
    sys.exit(1)

DOCS_DIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# FIND SOURCE FILES
# ============================================================

def find_source_files():

    cpp_files = sorted(SOURCE_DIR.glob("*.cpp"))
    hpp_files = sorted(SOURCE_DIR.glob("*.hpp"))

    return cpp_files, hpp_files


# ============================================================
# FIND RELATED HEADERS
# ============================================================

def find_related_headers(cpp_file, all_headers):

    related = set()

    # 1. Matching header name
    matching_header = cpp_file.with_suffix(".hpp")

    if matching_header.exists():
        related.add(matching_header.resolve())

    # 2. Detect locally included headers
    source = cpp_file.read_text(
        encoding="utf-8",
        errors="replace"
    )

    includes = re.findall(
        r'#include\s*"([^"]+)"',
        source
    )

    for include in includes:

        candidate = (cpp_file.parent / include).resolve()

        if candidate.is_file() and candidate.suffix == ".hpp":
            related.add(candidate)

        # Also search directly inside src
        for header in all_headers:

            if header.name.lower() == Path(include).name.lower():
                related.add(header.resolve())

    return sorted(related)


# ============================================================
# READ SOURCE FILES
# ============================================================

def read_source_file(file):

    return file.read_text(
        encoding="utf-8",
        errors="replace"
    )


# ============================================================
# GENERATE AI DOCUMENTATION
# ============================================================

def generate_documentation(source_file, related_headers):

    source_code = read_source_file(source_file)

    header_content = ""

    for header in related_headers:

        header_content += f"""

HEADER FILE: {header.name}

```cpp
{read_source_file(header)}
```

"""

    prompt = f"""
You are a senior C++ software architect,
DSP engineer and VCV Rack 2 developer.

You are documenting the AGMM VCV Rack plugin.

Analyze the complete source implementation and
all supplied header declarations together.

PROJECT: AGMM
PLATFORM: VCV Rack 2
LANGUAGE: C++

MAIN SOURCE FILE: {source_file.name}

RELATED HEADER FILES:
{", ".join(h.name for h in related_headers) or "None"}

Generate detailed, technically accurate Markdown.

DOCUMENTATION STRUCTURE:

# {source_file.stem}

## 1. Overview

Explain the purpose and functionality of this module.

## 2. Architecture

Explain:
- Main classes
- Inheritance
- Structures
- Relationships between classes
- Module organization

## 3. DSP Theory

Explain the mathematics and signal processing.

Include:
- Mathematical equations using LaTeX
- Algorithms
- Filters
- Oscillators
- Delay lines
- Feedback systems
- Modulation
- Random processes

Only discuss what actually exists in the code.

## 4. Parameters and Controls

Document all:
- Knobs
- Switches
- Buttons
- CV inputs
- Gates
- Enumerations
- Parameter ranges
- Default values

## 5. Inputs and Outputs

Explain every input and output.

Include voltage ranges and signal behavior
where these can be established from the source.

## 6. Signal Processing Pipeline

Explain the processing sequence step by step.

## 7. Important Functions

Explain important functions and their responsibilities.

## 8. Mathematical Implementation

Explain the actual mathematical implementation,
including equations and numerical methods.

## 9. Performance and Optimization

Explain:
- CPU usage considerations
- Memory allocation
- Buffers
- Real-time processing
- Thread safety
- Numerical stability

Only mention relevant implementation details.

## 10. Dependencies

List important libraries, classes and headers.

## 11. Summary

Provide a concise technical summary.

IMPORTANT REQUIREMENTS:

1. Read the CPP and HPP together.
2. Header declarations must be compared with implementation.
3. Never invent missing functionality.
4. Never assume a parameter exists.
5. Explain actual algorithms, not generic DSP theory.
6. Use correct C++ terminology.
7. Use professional Markdown.
8. If something is not applicable, say so.
9. Distinguish implemented behavior from inferred behavior.
10. Do not reproduce the entire source code.

MAIN SOURCE:

```cpp
{source_code}
```

RELATED HEADERS:

{header_content}
"""

    response = requests.post(
        OLLAMA_URL,
        json={
            "model": MODEL,
            "prompt": prompt,
            "stream": False,
            "keep_alive": "5m",
            "options": {
                "temperature": TEMPERATURE,
                "num_ctx": NUM_CTX
            }
        },
        timeout=1800
    )

    response.raise_for_status()

    return response.json()["response"]


# ============================================================
# PROCESS ONE FILE
# ============================================================

def process_file(source_file, related_headers=None):

    if related_headers is None:
        related_headers = []

    output_file = DOCS_DIR / (source_file.stem + ".md")

    print("\n" + "=" * 65)
    print("SOURCE:", source_file.name)

    if related_headers:
        print("RELATED HEADERS:")

        for header in related_headers:
            print("  +", header.name)

    print("=" * 65)

    try:

        documentation = generate_documentation(
            source_file,
            related_headers
        )

        if not documentation.strip():
            raise ValueError("Empty Ollama response")

        # Add source information
        metadata = f"""<!--
AGMM AUTOMATIC DOCUMENTATION
Source: {source_file.name}
Generated: {datetime.now().strftime("%Y-%m-%d %H:%M")}
-->

"""

        output_file.write_text(
            metadata + documentation,
            encoding="utf-8"
        )

        print("SUCCESS:", output_file.name)

        return True

    except requests.exceptions.ConnectionError:

        print("ERROR: Ollama is not running!")

    except requests.exceptions.Timeout:

        print("ERROR: Ollama request timed out!")

    except Exception as error:

        print("ERROR:", error)

    return False


# ============================================================
# EXTRACT DESCRIPTION FROM DOCUMENTATION
# ============================================================

def extract_description(markdown_file):

    content = markdown_file.read_text(
        encoding="utf-8",
        errors="replace"
    )

    match = re.search(
        r"##\s+1\.\s+Overview\s+(.*?)(?=\n##\s|\Z)",
        content,
        re.DOTALL | re.IGNORECASE
    )

    if match:

        description = match.group(1).strip()

        # Remove Markdown formatting
        description = re.sub(
            r"[#*`]",
            "",
            description
        )

        description = re.sub(
            r"\s+",
            " ",
            description
        )

        if description:
            return description[:220].strip()

    return "Technical documentation for this AGMM component."


# ============================================================
# GENERATE MAIN DOCUMENTATION INDEX
# ============================================================

def generate_index(cpp_files, hpp_files, successful):

    lines = []

    lines.append("# AGMM Documentation")
    lines.append("")
    lines.append(
        "Technical documentation for the AGMM VCV Rack 2 plugin."
    )

    lines.append("")
    lines.append("## About AGMM")
    lines.append("")
    lines.append(
        "AGMM is a collection of custom VCV Rack modules "
        "implemented in C++."
    )

    lines.append("")
    lines.append("## Module Documentation")
    lines.append("")
    lines.append("| Module | Description |")
    lines.append("|---|---|")

    for cpp_file in cpp_files:

        doc_file = DOCS_DIR / (cpp_file.stem + ".md")

        if not doc_file.exists():
            continue

        description = extract_description(doc_file)

        lines.append(
            f"| [{cpp_file.stem}]({cpp_file.stem}.md) "
            f"| {description} |"
        )

    lines.append("")
    lines.append("## Standalone Header Documentation")
    lines.append("")

    for hpp_file in hpp_files:

        doc_file = DOCS_DIR / (hpp_file.stem + ".md")

        # Only list headers with their own documentation
        # that are not already covered by a CPP.
        if not doc_file.exists():
            continue

        if any(cpp.stem == hpp_file.stem for cpp in cpp_files):
            continue

        lines.append(
            f"- [{hpp_file.stem}]({hpp_file.stem}.md)"
        )

    lines.append("")
    lines.append("## Documentation Generation")
    lines.append("")
    lines.append("- Generator: Python")
    lines.append(f"- AI model: `{MODEL}`")
    lines.append("- Backend: Ollama")
    lines.append("- Source language: C++")
    lines.append("- Documentation format: Markdown")
    lines.append("")
    lines.append("## Generation Statistics")
    lines.append("")
    lines.append(f"- C++ files discovered: {len(cpp_files)}")
    lines.append(f"- Header files discovered: {len(hpp_files)}")
    lines.append(f"- Successful documentation files: {successful}")
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("Automatically generated documentation for AGMM.")

    index_file = DOCS_DIR / "README.md"

    index_file.write_text(
        "\n".join(lines),
        encoding="utf-8"
    )

    print("\nMAIN INDEX CREATED:", index_file)


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 65)
    print("AGMM DOCUMENTATION GENERATOR")
    print("Ollama + Qwen3-Coder")
    print("=" * 65)

    print("\nProject:", PROJECT_DIR)
    print("Source:", SOURCE_DIR)
    print("Output:", DOCS_DIR)
    print("Model:", MODEL)

    cpp_files, hpp_files = find_source_files()

    print("\nCPP FILES:", len(cpp_files))
    print("HPP FILES:", len(hpp_files))

    if not cpp_files and not hpp_files:

        print("\nNo source files found!")
        sys.exit(1)

    successful = 0
    failed = 0

    # Track headers already covered by CPP documentation
    used_headers = set()

    # --------------------------------------------------------
    # STEP 1: DOCUMENT CPP FILES WITH RELATED HEADERS
    # --------------------------------------------------------

    for index, cpp_file in enumerate(cpp_files, 1):

        print(f"\n[{index}/{len(cpp_files)}]")

        related_headers = find_related_headers(
            cpp_file,
            hpp_files
        )

        for header in related_headers:
            used_headers.add(header.resolve())

        if process_file(cpp_file, related_headers):
            successful += 1
        else:
            failed += 1

    # --------------------------------------------------------
    # STEP 2: DOCUMENT STANDALONE HEADERS
    # --------------------------------------------------------

    standalone_headers = [
        header for header in hpp_files
        if header.resolve() not in used_headers
    ]

    for index, header in enumerate(standalone_headers, 1):

        print(f"\n[HEADER {index}/{len(standalone_headers)}]")

        if process_file(header):
            successful += 1
        else:
            failed += 1

    # --------------------------------------------------------
    # STEP 3: GENERATE MAIN README
    # --------------------------------------------------------

    generate_index(
        cpp_files,
        hpp_files,
        successful
    )

    # --------------------------------------------------------
    # FINAL REPORT
    # --------------------------------------------------------

    print("\n" + "=" * 65)
    print("AGMM DOCUMENTATION COMPLETE")
    print("=" * 65)

    print("CPP files:", len(cpp_files))
    print("HPP files:", len(hpp_files))
    print("Successful:", successful)
    print("Failed:", failed)

    print("\nDocumentation:", DOCS_DIR)
    print("Main index:", DOCS_DIR / "README.md")

    print("=" * 65)


if __name__ == "__main__":
    main()
