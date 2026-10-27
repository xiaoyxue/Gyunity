# Agent Guidelines

## Make changes in a worktree

Do not edit files in the main checkout. Create a git worktree under `.worktree/` at the repository root and make all changes there.

```powershell
git fetch origin
git worktree add .worktree/<topic> -b user/<alias>/<topic> origin/main
cd .worktree/<topic>
git submodule update --init --recursive
.\init.ps1
```

- Name the worktree directory after the topic, and the branch `user/<alias>/<topic>`.
- A new worktree has no submodules and no `external/build`, so run the two setup commands above before building.
- `.worktree/` is ignored by git; never commit anything under it.
- When the branch is merged, remove the worktree with `git worktree remove .worktree/<topic>`.

## Test output

Write all test output (renders, logs, scratch scripts) to `Test/` at the main repository root, not inside a worktree. Everything in it except `Test/.gitignore` is ignored by git.

## Error handling

- Do not use C++ exceptions for normal control flow or error handling in repository-owned engine, library or helper code. Do not introduce `throw` or `try`/`catch` error paths there.
- `GyunityApp/main.cpp` may have one outermost `try`/`catch` boundary to report unexpected standard-library or dependency exceptions and return a nonzero exit code. Keep expected input/I/O failures on explicit status paths; do not add throwing helpers or broad catch-and-continue behavior.
- Return explicit success/error status values and propagate failures to the caller. Use non-throwing logging, such as `GetDefaultLogger()->Error(message, false)`.
- Prefer checked, non-throwing interfaces when integrating dependencies. Do not hide failures behind empty successful results or silently ignore errors.
- Python may use normal validation exceptions.
