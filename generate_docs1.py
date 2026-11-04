
import requests
import re
import sys
import time

from pathlib import Path
from datetime import datetime

# ============================================================
# AGMM DEEP DOCUMENTATION GENERATOR
# Qwen3-Coder 30B + Ollama
# VCV Rack 2
# ============================================================

MODEL = "qwen3-coder:30b"

PROJECT_DIR = Path(__file__).resolve().parent
SOURCE_DIR = PROJECT_DIR / "src"
DOCS_DIR = PROJECT_DIR / "docs"

OLLAMA_URL = "http://localhost:11434/api/generate"

# AI configuration
NUM_CTX = 16384
NUM_PREDICT = 2500
TEMPERATURE = 0.2

# Maximum characters per source chunk
CHUNK_SIZE = 10000

# Number of analysis reports combined at a time
REDUCE_BATCH_SIZE = 4

# Supported files
CPP_EXTENSIONS = {".cpp"}
HEADER_EXTENSIONS = {".hpp"}

# ============================================================
# INITIALIZATION
# ============================================================

if not SOURCE_DIR.is_dir():
    print("ERROR: src directory not found!")
    print(SOURCE_DIR)
    sys.exit(1)

DOCS_DIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# OLLAMA API
# ============================================================

def ask_ollama(prompt):

    response = requests.post(
        OLLAMA_URL,
        json={
            "model": MODEL,
            "prompt": prompt,
            "stream": False,
            "keep_alive": "5m",
            "options": {
                "temperature": TEMPERATURE,
                "num_ctx": NUM_CTX,
                "num_predict": NUM_PREDICT
            }
        },
        timeout=1800
    )

    response.raise_for_status()

    result = response.json().get("response", "").strip()

    if not result:
        raise RuntimeError("Ollama returned an empty response")

    return result


# ============================================================
# FIND SOURCE FILES
# ============================================================

def find_source_files():

    cpp_files = sorted(
        f for f in SOURCE_DIR.rglob("*")
        if f.is_file() and f.suffix.lower() in CPP_EXTENSIONS
    )

    header_files = sorted(
        f for f in SOURCE_DIR.rglob("*")
        if f.is_file() and f.suffix.lower() in HEADER_EXTENSIONS
    )

    return cpp_files, header_files


# ============================================================
# FIND RELATED HEADERS
# ============================================================

def find_related_headers(cpp_file, all_headers):

    related = {}

    # Matching filename: Module.cpp -> Module.hpp
    matching = cpp_file.with_suffix(".hpp")

    if matching.is_file():
        related[matching.resolve()] = matching

    # Detect local #include "..." directives
    source = cpp_file.read_text(
        encoding="utf-8",
        errors="replace"
    )

    includes = re.findall(
        r'^\s*#\s*include\s*"([^"]+)"',
        source,
        re.MULTILINE
    )

    for include in includes:

        include_name = Path(include).name

        for header in all_headers:

            if header.name.lower() == include_name.lower():
                related[header.resolve()] = header

    return sorted(
        related.values(),
        key=lambda f: str(f).lower()
    )


# ============================================================
# SPLIT SOURCE INTO CHUNKS
# ============================================================

def split_into_chunks(source, max_chars=CHUNK_SIZE):

    lines = source.splitlines()

    chunks = []
    current = []
    current_size = 0

    start_line = 1

    for line_number, line in enumerate(lines, start=1):

        line_size = len(line) + 1

        # Handle exceptionally long individual lines
        if line_size > max_chars:

            if current:
                chunks.append({
                    "start": start_line,
                    "end": line_number - 1,
                    "text": "\n".join(current)
                })

                current = []
                current_size = 0

            for offset in range(0, len(line), max_chars):

                fragment = line[offset:offset + max_chars]

                chunks.append({
                    "start": line_number,
                    "end": line_number,
                    "text": fragment
                })

            start_line = line_number + 1
            continue

        if current and current_size + line_size > max_chars:

            chunks.append({
                "start": start_line,
                "end": line_number - 1,
                "text": "\n".join(current)
            })

            current = []
            current_size = 0
            start_line = line_number

        current.append(line)
        current_size += line_size

    if current:

        chunks.append({
            "start": start_line,
            "end": len(lines),
            "text": "\n".join(current)
        })

    return chunks


# ============================================================
# STAGE 1: DEEP SOURCE ANALYSIS
# ============================================================

def analyze_source_chunk(file, chunk, total_chunks, index):

    print(
        f"    Analyzing {file.name}: "
        f"chunk {index}/{total_chunks} "
        f"(lines {chunk['start']}-{chunk['end']})"
    )

    prompt = f"""
You are a senior C++ software architect,
digital signal processing engineer and VCV Rack 2 developer.

You are analyzing the source code of the AGMM plugin.

Perform a detailed technical examination of this
specific section of the source code.

FILE: {file.name}

FILE TYPE: {file.suffix}

CHUNK: {index}/{total_chunks}

SOURCE LINES: {chunk['start']}-{chunk['end']}

ANALYZE THE FOLLOWING:

1. CLASSES AND STRUCTURES
   - Class declarations and responsibilities.
   - Inheritance and relationships.
   - Member variables and their purpose.
   - State management.

2. FUNCTIONS
   - Every function visible in this section.
   - Function arguments and return values.
   - Processing sequence.
   - Side effects and state changes.
   - Calls to other functions.
   - Initialization and cleanup.

3. DSP MATHEMATICS
   - Mathematical equations actually implemented.
   - Filters, oscillators and modulation.
   - Feedback and delay calculations.
   - Numerical methods.
   - Random and chaotic algorithms.
   - Interpolation and smoothing.
   - Stability and clipping.

4. VCV RACK IMPLEMENTATION
   - Parameters and parameter IDs.
   - Inputs and outputs.
   - Voltage calculations.
   - Polyphonic processing.
   - Process callbacks.
   - Rack API usage.

5. MEMORY AND PERFORMANCE
   - Buffers and arrays.
   - Memory allocation.
   - Persistent state.
   - Potential real-time concerns.
   - Computational complexity where determinable.

6. IMPLEMENTATION DETAILS
   - Explain important code blocks in execution order.
   - Explain conditions, loops and calculations.
   - Explain how values are transformed.
   - Explain dependencies and interactions.

7. POTENTIAL ISSUES
   - Possible edge cases.
   - Numerical risks.
   - Possible bugs.
   - Performance bottlenecks.

IMPORTANT:

- Analyze only the supplied source.
- Do not invent missing functions or functionality.
- Distinguish confirmed behavior from assumptions.
- If something requires another file, explicitly say so.
- Preserve actual parameter names and IDs.
- Include source line references where possible.
- Explain mathematical equations accurately.
- Be detailed rather than superficial.

SOURCE CODE:

{chunk['text']}
"""

    return ask_ollama(prompt)


# ============================================================
# ANALYZE ONE COMPLETE FILE
# ============================================================

def analyze_file(file, role):

    source = file.read_text(
        encoding="utf-8",
        errors="replace"
    )

    chunks = split_into_chunks(source)

    print(f"\n  FILE: {file.name}")
    print(f"  ROLE: {role}")
    print(f"  SIZE: {len(source):,} characters")
    print(f"  CHUNKS: {len(chunks)}")

    results = []

    for index, chunk in enumerate(chunks, start=1):

        analysis = analyze_source_chunk(
            file,
            chunk,
            len(chunks),
            index
        )

        results.append({
            "file": file.name,
            "role": role,
            "start": chunk["start"],
            "end": chunk["end"],
            "analysis": analysis
        })

    return results


# ============================================================
# STAGE 2A: REDUCE ANALYSIS RESULTS
# ============================================================

def reduce_analysis(batch):

    combined = []

    for item in batch:

        combined.append(
            f"""
FILE: {item['file']}
ROLE: {item['role']}
LINES: {item['start']}-{item['end']}

ANALYSIS:
{item['analysis']}
"""
        )

    prompt = f"""
You are consolidating technical analysis
for the AGMM VCV Rack 2 C++ plugin.

Combine the following analysis reports into
a comprehensive technical report.

Preserve:

- All identified functions.
- All classes and structures.
- All parameter names and IDs.
- All inputs and outputs.
- All mathematical equations.
- All algorithm descriptions.
- All implementation details.
- All source line references.
- All potential issues.

Remove repetitive explanations, but NEVER remove
unique technical information.

Do not invent functionality.

ANALYSIS REPORTS:

{"".join(combined)}
"""

    return ask_ollama(prompt)


# ============================================================
# STAGE 2B: HIERARCHICAL ANALYSIS REDUCTION
# ============================================================

def consolidate_analysis(analyses):

    current = analyses
    level = 1

    while len(current) > REDUCE_BATCH_SIZE:

        print(
            f"\n  Consolidation level {level}: "
            f"{len(current)} reports"
        )

        reduced = []

        for start in range(
            0,
            len(current),
            REDUCE_BATCH_SIZE
        ):

            batch = current[
                start:start + REDUCE_BATCH_SIZE
            ]

            result = reduce_analysis(batch)

            reduced.append({
                "file": "Consolidated analysis",
                "role": "Technical analysis",
                "start": 0,
                "end": 0,
                "analysis": result
            })

        current = reduced
        level += 1

    return reduce_analysis(current) if current else ""


# ============================================================
# STAGE 2C: FINAL DOCUMENTATION GENERATION
# ============================================================

def generate_final_documentation(cpp_file, headers, analysis):

    header_names = ", ".join(
        header.name for header in headers
    )

    prompt = f"""
You are the lead C++ architect and DSP documentation
specialist for the AGMM VCV Rack 2 plugin.

Create an exhaustive, professional Markdown
technical document for this module.

MAIN IMPLEMENTATION:
{cpp_file.name}

RELATED HEADERS:
{header_names or "No related headers found"}

BASE YOUR DOCUMENTATION ON THE ANALYSIS REPORT
PROVIDED BELOW.

REQUIRED DOCUMENT STRUCTURE:

# {cpp_file.stem}

## 1. Module Overview

Explain the module's complete purpose, behavior
and intended functionality.

## 2. Architecture and Design

Explain classes, structures, inheritance,
member variables and relationships.

## 3. Complete Function Reference

Document every identifiable function.

For each function include:
- Function name and signature.
- Purpose.
- Arguments.
- Return value.
- Internal operations.
- State modifications.
- Calls to other functions.

## 4. Implementation Analysis

Explain the important code blocks in execution order.

Include initialization, processing, conditions,
loops, state transitions and calculations.

Reference source line numbers where available.

## 5. DSP Theory and Mathematics

Explain the actual mathematics used.

Include:
- Equations in LaTeX.
- Derivation where possible.
- Explanation of variables.
- Sampling and timing behavior.
- Filters and modulation.
- Feedback and delay calculations.
- Numerical methods.
- Stability considerations.

Do not introduce algorithms that are not implemented.

## 6. Parameter Reference

Create a table with:

| Parameter | ID | Range | Default | Function |

Document every parameter that can be established
from the source.

## 7. Input Reference

Create a table describing every input:

| Input | ID | Signal Type | Voltage Behavior | Function |

## 8. Output Reference

Create a table describing every output:

| Output | ID | Signal Type | Voltage Behavior | Function |

## 9. Signal Processing Pipeline

Explain the complete signal path, from inputs
through internal processing to outputs.

## 10. Polyphony and CV Behavior

Explain channel handling, polyphony, CV modulation,
gate behavior and channel expansion where applicable.

## 11. Memory and State Management

Explain buffers, arrays, variables, initialization,
persistent state and reset behavior.

## 12. Performance and Optimization

Analyze CPU complexity, memory usage,
real-time processing, allocations and optimization.

Only make performance claims supported by the code.

## 13. VCV Rack API Integration

Explain Rack-specific callbacks, module lifecycle,
parameter configuration and registration where present.

## 14. Potential Issues and Improvements

Separate:
- Confirmed issues.
- Potential risks.
- Suggested improvements.

Never report a hypothetical problem as a confirmed bug.

## 15. Technical Summary

Provide a detailed final summary.

## 16. Source Files

List the implementation and related header files.

DOCUMENTATION QUALITY:

- Be exhaustive and technically precise.
- Explain every identifiable function.
- Do not skip important implementation details.
- Do not invent functionality.
- Preserve exact identifiers.
- Distinguish facts from inferences.
- Use professional Markdown.
- Prefer equations and tables where appropriate.
- Do not simply repeat the analysis report.

TECHNICAL ANALYSIS REPORT:

{analysis}
"""

    return ask_ollama(prompt)


# ============================================================
# PROCESS ONE CPP MODULE
# ============================================================

def process_module(cpp_file, all_headers):

    print("\n" + "=" * 70)
    print("MODULE:", cpp_file.stem)
    print("=" * 70)

    related_headers = find_related_headers(
        cpp_file,
        all_headers
    )

    print("\nRELATED HEADERS:")

    if related_headers:
        for header in related_headers:
            print("  +", header.name)
    else:
        print("  None")

    # Analyze implementation
    analyses = analyze_file(
        cpp_file,
        "C++ implementation"
    )

    # Analyze related headers
    for header in related_headers:

        header_analyses = analyze_file(
            header,
            "C++ header/interface"
        )

        analyses.extend(header_analyses)

    # Consolidate all analysis
    print("\n  CONSOLIDATING TECHNICAL ANALYSIS...")

    complete_analysis = consolidate_analysis(analyses)

    # Generate final document
    print("\n  GENERATING FINAL DOCUMENTATION...")

    documentation = generate_final_documentation(
        cpp_file,
        related_headers,
        complete_analysis
    )

    if not documentation.strip():
        raise RuntimeError("Empty final documentation")

    # Save Markdown
    output_file = DOCS_DIR / f"{cpp_file.stem}.md"

    metadata = f"""<!--
AGMM TECHNICAL DOCUMENTATION
Source: {cpp_file.name}
Headers: {", ".join(h.name for h in related_headers)}
Model: {MODEL}
Generated: {datetime.now().strftime("%Y-%m-%d %H:%M")}
-->

"""

    output_file.write_text(
        metadata + documentation,
        encoding="utf-8"
    )

    print("\n  CREATED:", output_file.name)

    return True


# ============================================================
# PROCESS STANDALONE HEADERS
# ============================================================

def process_standalone_header(header):

    print("\n" + "=" * 70)
    print("STANDALONE HEADER:", header.name)
    print("=" * 70)

    analyses = analyze_file(
        header,
        "Standalone C++ header/interface"
    )

    complete_analysis = consolidate_analysis(analyses)

    documentation = generate_final_documentation(
        header,
        [],
        complete_analysis
    )

    output_file = DOCS_DIR / f"{header.stem}.md"

    metadata = f"""<!--
AGMM HEADER DOCUMENTATION
Source: {header.name}
Model: {MODEL}
Generated: {datetime.now().strftime("%Y-%m-%d %H:%M")}
-->

"""

    output_file.write_text(
        metadata + documentation,
        encoding="utf-8"
    )

    print("\n  CREATED:", output_file.name)

    return True


# ============================================================
# EXTRACT DESCRIPTION
# ============================================================

def extract_description(markdown_file):

    content = markdown_file.read_text(
        encoding="utf-8",
        errors="replace"
    )

    match = re.search(
        r"##\s+1\.\s+Module Overview\s+(.*?)(?=\n##\s|\Z)",
        content,
        re.DOTALL | re.IGNORECASE
    )

    if not match:
        match = re.search(
            r"##\s+1\.\s+Overview\s+(.*?)(?=\n##\s|\Z)",
            content,
            re.DOTALL | re.IGNORECASE
        )

    if not match:
        return "Technical documentation."

    description = match.group(1).strip()

    description = re.sub(
        r"<!--.*?-->",
        "",
        description,
        flags=re.DOTALL
    )

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

    return description[:250].strip()


# ============================================================
# STAGE 3: GENERATE MASTER README
# ============================================================

def generate_master_readme(cpp_files, header_files, successful):

    print("\nGENERATING MASTER README...")

    lines = []

    lines.append("# AGMM Documentation")
    lines.append("")
    lines.append(
        "Complete technical documentation for the "
        "AGMM VCV Rack 2 plugin."
    )

    lines.append("")
    lines.append("## Project Overview")
    lines.append("")
    lines.append(
        "AGMM is a collection of custom VCV Rack modules "
        "implemented in C++."
    )

    lines.append("")
    lines.append("## Module Index")
    lines.append("")
    lines.append("| Module | Description |")
    lines.append("|---|---|")

    documented = []

    for cpp_file in cpp_files:

        doc_file = DOCS_DIR / f"{cpp_file.stem}.md"

        if not doc_file.exists():
            continue

        description = extract_description(doc_file)

        lines.append(
            f"| [{cpp_file.stem}]({cpp_file.stem}.md) "
            f"| {description} |"
        )

        documented.append(doc_file)

    # Standalone headers
    lines.append("")
    lines.append("## Standalone Header Documentation")
    lines.append("")

    cpp_stems = {f.stem.lower() for f in cpp_files}

    for header in header_files:

        if header.stem.lower() in cpp_stems:
            continue

        doc_file = DOCS_DIR / f"{header.stem}.md"

        if doc_file.exists():

            lines.append(
                f"- [{header.stem}]({header.stem}.md)"
            )

            documented.append(doc_file)

    # Technical information
    lines.append("")
    lines.append("## Documentation Statistics")
    lines.append("")

    lines.append(f"- C++ implementation files: {len(cpp_files)}")
    lines.append(f"- Header files: {len(header_files)}")
    lines.append(f"- Documents generated: {successful}")
    lines.append(f"- Documents indexed: {len(documented)}")
    lines.append(f"- AI model: `{MODEL}`")
    lines.append("- Backend: Ollama")
    lines.append("- Documentation format: Markdown")

    lines.append("")
    lines.append("## Documentation Generation")
    lines.append("")
    lines.append("Generated using `generate_docs.py`.")
    lines.append("")
    lines.append(
        "Each module document is produced by analyzing "
        "its C++ implementation and related headers."
    )

    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("AGMM - Automatic Technical Documentation")

    readme = DOCS_DIR / "README.md"

    readme.write_text(
        "\n".join(lines),
        encoding="utf-8"
    )

    print("MASTER README CREATED:", readme)


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 70)
    print("AGMM DEEP DOCUMENTATION GENERATOR")
    print("Ollama + Qwen3-Coder 30B")
    print("=" * 70)

    print("\nPROJECT:", PROJECT_DIR)
    print("SOURCE:", SOURCE_DIR)
    print("OUTPUT:", DOCS_DIR)
    print("MODEL:", MODEL)

    cpp_files, header_files = find_source_files()

    print("\nCPP FILES:", len(cpp_files))
    print("HPP FILES:", len(header_files))

    if not cpp_files and not header_files:
        print("\nNo source files found!")
        sys.exit(1)

    # Check Ollama connection before processing
    print("\nChecking Ollama connection...")

    try:

        requests.get(
            "http://localhost:11434/api/tags",
            timeout=10
        ).raise_for_status()

        print("Ollama is available.")

    except requests.RequestException as error:

        print("ERROR: Cannot connect to Ollama.")
        print(error)
        sys.exit(1)

    successful = 0
    failed = 0

    used_headers = set()

    # --------------------------------------------------------
    # STEP 1: DOCUMENT EVERY CPP + RELATED HPP
    # --------------------------------------------------------

    for index, cpp_file in enumerate(cpp_files, 1):

        print(f"\nMODULE [{index}/{len(cpp_files)}]")

        try:

            related = find_related_headers(
                cpp_file,
                header_files
            )

            for header in related:
                used_headers.add(header.resolve())

            process_module(
                cpp_file,
                header_files
            )

            successful += 1

        except KeyboardInterrupt:

            print("\nGeneration interrupted by user.")
            break

        except Exception as error:

            failed += 1

            print("\nMODULE FAILED:", cpp_file.name)
            print("ERROR:", error)

    # --------------------------------------------------------
    # STEP 2: DOCUMENT HEADERS WITHOUT CPP COUNTERPARTS
    # --------------------------------------------------------

    standalone_headers = [
        header for header in header_files
        if header.resolve() not in used_headers
    ]

    for index, header in enumerate(standalone_headers, 1):

        print(
            f"\nSTANDALONE HEADER "
            f"[{index}/{len(standalone_headers)}]"
        )

        try:

            process_standalone_header(header)
            successful += 1

        except KeyboardInterrupt:

            print("\nGeneration interrupted by user.")
            break

        except Exception as error:

            failed += 1

            print("\nHEADER FAILED:", header.name)
            print("ERROR:", error)

    # --------------------------------------------------------
    # STEP 3: GENERATE MASTER README
    # --------------------------------------------------------

    generate_master_readme(
        cpp_files,
        header_files,
        successful
    )

    # --------------------------------------------------------
    # FINAL REPORT
    # --------------------------------------------------------

    print("\n" + "=" * 70)
    print("AGMM DOCUMENTATION GENERATION FINISHED")
    print("=" * 70)

    print("CPP files:", len(cpp_files))
    print("HPP files:", len(header_files))
    print("Successful:", successful)
    print("Failed:", failed)

    print("\nDOCUMENTATION DIRECTORY:")
    print(DOCS_DIR)

    print("\nMASTER README:")
    print(DOCS_DIR / "README.md")

    print("=" * 70)


if __name__ == "__main__":
    main()
