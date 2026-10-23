# Agent Guidelines

## Make changes in a worktree

Do not edit files in the main checkout. Create a git worktree under `.worktree/` at the repository root and make all changes there.

```powershell
git fetch origin
git worktree add .worktree/<topic> -b user/<alias>/<topic> origin/master
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
