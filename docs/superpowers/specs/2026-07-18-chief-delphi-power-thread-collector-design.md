# Chief Delphi Power Thread Collector Design

**Date:** 2026-07-18
**Status:** Approved

## Goal

Create a command-line script that finds Chief Delphi threads whose discussions
mention brownouts, power draw, or closely related electrical-load concepts. The
script writes every unique matching thread URL to a text file, one URL per line.

Here, "matching" means that a configured search phrase appears in any post in a
thread, not only in the thread title. Completeness is bounded by Chief Delphi's
public Discourse search index and API behavior; the script does not claim to
find content the site does not return.

## Scope

The implementation adds:

- `scripts/find_chief_delphi_power_threads.py`
- `scripts/tests/test_find_chief_delphi_power_threads.py`

The script uses only the Python standard library. It does not change the robot
code, Gradle build, or project dependencies.

The default output is `chief_delphi_power_threads.txt` in the current working
directory.

## Search Coverage

The default phrase list covers:

- `brownout`
- `brown out`
- `browning out`
- `brown-out`
- `power draw`
- `current draw`
- `power usage`
- `power consumption`
- `voltage sag`
- `battery sag`
- `current limit`
- `current limiting`

The phrases are defined in one editable constant. A repeatable `--phrase`
option adds phrases for a run without removing the defaults.
Multi-word phrases are submitted as quoted Discourse searches so that broad
terms such as `power` and `draw` do not match independently.

The script uses Chief Delphi's public Discourse search endpoint. Discourse
documents public content searches through `/search.json?q={search_term}` and
supports paginating that route:

- https://meta.discourse.org/t/admin-dashboard-report-reference-guide/240233/40
- https://meta.discourse.org/t/discourses-api-get-just-the-number-of-search-results/76548/7

## Architecture

The script separates four responsibilities:

1. Build search request URLs and fetch JSON with retry handling.
2. Traverse all result pages for each phrase.
3. Extract topic identities and canonical URLs, deduplicating by topic ID.
4. Atomically write the final sorted URL set.

The HTTP transport is passed into the collection logic. Production uses a
standard-library URL opener; tests use deterministic in-memory responses.

Canonical URLs have this form:

`https://www.chiefdelphi.com/t/{topic-slug}/{topic-id}`

Results are sorted by numeric topic ID so repeated runs against the same search
state produce stable output.

## Command-Line Interface

The default command is:

```text
python scripts/find_chief_delphi_power_threads.py
```

Supported options are:

- `--output PATH`: destination file; defaults to
  `chief_delphi_power_threads.txt`.
- `--delay SECONDS`: nonnegative pause between requests; defaults to a polite
  one-second delay.
- `--phrase TEXT`: repeatable additional search phrase.

On success, the command prints the number of unique thread URLs written and
returns exit code zero. On failure, it prints a concise error to standard error
and returns a nonzero exit code.

## Data Flow

For each configured phrase, the script:

1. Requests search-result page 1.
2. Extracts the posts and corresponding topics from the response.
3. Adds each topic to a map keyed by numeric topic ID.
4. Requests the next numbered page while
   `grouped_search_result.more_full_page_results` is `true`.
5. Records page identities from returned post IDs and stops with an error if
   the site repeats a page while still reporting more results.

After every phrase completes successfully, the script builds canonical URLs,
sorts them by topic ID, and writes the destination.

## Failure Handling and Site Courtesy

Every request includes a descriptive user agent. The configured delay applies
between requests.

The fetcher retries temporary network failures, HTTP 429 responses, and
temporary HTTP 5xx responses with bounded backoff. A valid `Retry-After` header
is honored. Permanent HTTP errors, malformed JSON, missing required response
fields, repeated pages, and exhausted retries fail the run with a clear
message.

Output uses a temporary file in the destination directory followed by an
atomic replacement. If collection or writing fails, the temporary file is
removed and any existing destination remains unchanged. This prevents a
partial result from appearing complete.

## Testing

Tests use Python's built-in `unittest` and cover:

- response parsing and canonical URL construction;
- pagination across multiple pages;
- deduplication across pages and phrases;
- retryable and permanent HTTP failures;
- repeated-page detection;
- stable numeric sorting;
- atomic output and preservation of an existing file after failure;
- command-line success and error behavior with temporary directories and
  deterministic responses.

Implementation follows test-driven development: each production behavior is
preceded by a test that fails for the expected reason.

After unit tests pass, verification includes a live smoke test against Chief
Delphi. The generated file is checked for nonempty, unique, well-formed Chief
Delphi topic URLs.

## Non-Goals

- Crawling every topic and post on Chief Delphi.
- Scraping rendered HTML pages.
- Using external search engines.
- Downloading thread content into the output file.
- Classifying relevance beyond the configured search phrases.
- Guaranteeing discovery of posts omitted by Chief Delphi's search index.
