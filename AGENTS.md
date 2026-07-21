# Workspace Rules

## One-way local, WSL, and GitHub synchronization

- Edit source files only in the local repository. Never make source edits in
  `/root/smartcar2026-simulation`.
- After local validation, commit the local changes and push them to GitHub.
- Fast-forward `/root/smartcar2026-simulation` from GitHub to the same commit;
  it is a deployment copy, not a second source workspace.
- Before reporting completion, verify that the local repository, GitHub branch,
  and root WSL workspace point to the same `HEAD` commit.
- If the root WSL workspace is not clean, stop and ask the user before syncing;
  never merge or copy WSL source changes back into the local repository.
- Do not synchronize generated `build/`, `devel/`, `log/`, `.ros/`, or Gazebo
  cache files. Build artifacts remain local to the WSL deployment.

## Global Rules

- Do not use Playwright MCP. When browser automation or Playwright testing is
  needed, use the Playwright CLI instead.
